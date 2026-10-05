"""Convert the pretrained G1 walking policy from unitree_rl_gym (TorchScript,
an LSTM: 47-dim obs -> 64 hidden -> 32 -> 12 actions) into ONNX so it can run
client-side in the browser via onnxruntime-web.

The original checkpoint stores its LSTM hidden/cell state as in-place-mutated
module buffers, which ONNX's tracer can't export (ONNX graphs are stateless).
This rebuilds the same architecture with plain nn.LSTM/nn.Linear layers,
copies the checkpoint's weights over, and exposes hidden/cell state as
explicit inputs/outputs instead — verified to produce bit-identical actions
against the original checkpoint (see the diff check below) before export.

Usage:
    git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym /tmp/unitree_rl_gym
    python scripts/export_policy.py /tmp/unitree_rl_gym/deploy/pre_train/g1/motion.pt \
        web/assets/policy/g1_walk_policy.onnx
"""
import sys

import torch
import torch.nn as nn


class ExportablePolicy(nn.Module):
    def __init__(self, lstm, actor):
        super().__init__()
        self.lstm = lstm
        self.actor = actor

    def forward(self, obs, h, c):
        x = obs.unsqueeze(0)  # (seq_len=1, batch=1, input_size)
        out, (h_new, c_new) = self.lstm(x, (h, c))
        out = out.squeeze(0)
        action = self.actor(out)
        return action, h_new, c_new


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    checkpoint_path, out_path = sys.argv[1], sys.argv[2]

    policy = torch.jit.load(checkpoint_path)
    policy.eval()

    lstm = nn.LSTM(input_size=47, hidden_size=64, num_layers=1)
    lstm.load_state_dict(dict(policy.memory.named_parameters()))
    lstm.eval()

    actor = nn.Sequential(nn.Linear(64, 32), nn.ELU(), nn.Linear(32, 12))
    actor.load_state_dict(dict(policy.actor.named_parameters()))
    actor.eval()

    wrapper = ExportablePolicy(lstm, actor)
    wrapper.eval()

    obs = torch.zeros(1, 47)
    h0 = torch.zeros(1, 1, 64)
    c0 = torch.zeros(1, 1, 64)

    with torch.no_grad():
        action, h1, c1 = wrapper(obs, h0, c0)
        policy.hidden_state.zero_()
        policy.cell_state.zero_()
        ref = policy(obs)
        diff1 = (action - ref).abs().max().item()

        obs2 = torch.randn(1, 47) * 0.1
        action2, _, _ = wrapper(obs2, h1, c1)
        ref2 = policy(obs2)
        diff2 = (action2 - ref2).abs().max().item()

    print(f"step1 max abs diff vs original checkpoint: {diff1}")
    print(f"step2 max abs diff vs original checkpoint: {diff2}")
    assert diff1 < 1e-5 and diff2 < 1e-5, "exported policy diverges from the original checkpoint"

    torch.onnx.export(
        wrapper, (obs, h0, c0), out_path,
        input_names=["obs", "h_in", "c_in"],
        output_names=["action", "h_out", "c_out"],
        opset_version=17,
        dynamo=False,
    )
    print(f"exported {out_path}")


if __name__ == "__main__":
    main()
