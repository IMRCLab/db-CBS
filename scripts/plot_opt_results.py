#!/usr/bin/env python3
import argparse
from pathlib import Path
import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)

def ensure_outdir(outdir):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    plots_dir = outdir / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    return plots_dir

def _axis_limits_from_data(Y, pad=0.1):
    m = np.min([np.min(y) for y in Y])
    M = np.max([np.max(y) for y in Y])
    if np.isclose(M, m):
        eps = 1.0 if np.isclose(M, 0.0) else 0.05*abs(M) + 1e-3
        return (m - eps, M + eps)
    span = M - m
    return (m - pad*span, M + pad*span)

def save_xyz_stack(t, Y3, title, units, outpath, dashed_limits=None, ylimits=None, pdf=None):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    labels = ['x', 'y', 'z']
    for ax, y, lab in zip(axes, Y3, labels):
        ax.plot(t, y, label=lab)
        ax.set_ylabel(f"{lab} [{units}]")
        if dashed_limits is not None:
            ymin, ymax = dashed_limits
            ax.axhline(ymin, linestyle="--", linewidth=1.0)
            ax.axhline(ymax, linestyle="--", linewidth=1.0)
            ax.set_ylim(ymin, ymax)
        elif ylimits is not None:
            ax.set_ylim(ylimits)
        else:
            ax.set_ylim(_axis_limits_from_data([y]))
        ax.grid(True, which='both', alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(title)
    fig.tight_layout(rect=[0,0,1,0.97])
    fig.savefig(outpath, dpi=150)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)

def save_inputs(t, U, title, outpath, max_f=None, pdf=None):
    fig, ax = plt.subplots(figsize=(9, 3.5))
    for j in range(U.shape[1]):
        ax.plot(t, U[:, j], label=f"u{j+1}")
    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("input")
    if max_f is not None and max_f > 0:
        ax.axhline(0.0, linestyle="--", linewidth=1.0)
        ax.axhline(max_f, linestyle="--", linewidth=1.0)
        margin = 0.02 * max_f
        ax.set_ylim(0.0 - margin, max_f + margin)
    ax.grid(True, which='both', alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outpath, dpi=150)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)

def save_single_series(t, y, label, title, ylabel, outpath, ylimits=None, pdf=None):
    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(t, y, label=label)
    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    if ylimits is not None:
        ax.set_ylim(ylimits)
    ax.grid(True, which='both', alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(outpath, dpi=150)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)

def parse_states(states, num_robots_hint=None):
    """
    Supports two layouts:
      A) payload + R quads:
         len = 13 + 13R  with order:
           payload: pos3 quat4, then each quad: pos3 quat4,
           then payload: vel3 ang3, then each quad: vel3 ang3
      B) quads only (no payload):
         len = 13R with order:
           for each quad: pos3 quat4 vel3 ang3
    If num_robots_hint is provided and consistent, it's used; otherwise inferred.
    Returns a dict with keys:
      has_payload (bool), R (int),
      p_pos, p_quat, p_vel, p_ang (or None if no payload),
      q_pos, q_quat, q_vel, q_ang (lists length R)
    """
    X = np.asarray(states, float)
    N, L = X.shape

    def ok_int(x): return abs(x - round(x)) < 1e-9

    # Try detect with hint
    if num_robots_hint is not None:
        R = int(num_robots_hint)
        if L == 13 + 13*R:
            # payload present
            return _parse_payload_plus_quads(X, R, has_payload=True)
        if L == 13*R:
            # quads only
            return _parse_quads_only(X, R, has_payload=False)
        # fall-through to inference

    # Infer R and layout
    if L >= 26 and ok_int((L - 13) / 13.0):  # could be payload + R quads
        R = int(round((L - 13) / 13.0))
        return _parse_payload_plus_quads(X, R, has_payload=True)
    if L >= 13 and ok_int(L / 13.0):         # could be pure quads
        R = int(round(L / 13.0))
        return _parse_quads_only(X, R, has_payload=False)

    raise ValueError(f"Unrecognized state layout length={L}. Provide a consistent model or data.")

def _parse_payload_plus_quads(X, R, has_payload=True):
    N, L = X.shape
    idx = 0
    p_pos  = X[:, idx:idx+3]; idx += 3
    p_quat = X[:, idx:idx+4]; idx += 4
    q_pos, q_quat = [], []
    for _ in range(R):
        q_pos.append(X[:, idx:idx+3]); idx += 3
        q_quat.append(X[:, idx:idx+4]); idx += 4
    p_vel = X[:, idx:idx+3]; idx += 3
    p_ang = X[:, idx:idx+3]; idx += 3
    q_vel, q_ang = [], []
    for _ in range(R):
        q_vel.append(X[:, idx:idx+3]); idx += 3
        q_ang.append(X[:, idx:idx+3]); idx += 3
    return dict(has_payload=True, R=R,
                p_pos=p_pos, p_quat=p_quat, p_vel=p_vel, p_ang=p_ang,
                q_pos=q_pos, q_quat=q_quat, q_vel=q_vel, q_ang=q_ang)

def _parse_quads_only(X, R, has_payload=False):
    N, L = X.shape
    q_pos, q_quat, q_vel, q_ang = [], [], [], []
    idx = 0
    for _ in range(R):
        q_pos.append(X[:, idx:idx+3]); idx += 3
        q_quat.append(X[:, idx:idx+4]); idx += 4
        q_vel.append(X[:, idx:idx+3]); idx += 3
        q_ang.append(X[:, idx:idx+3]); idx += 3
    return dict(has_payload=False, R=R,
                p_pos=None, p_quat=None, p_vel=None, p_ang=None,
                q_pos=q_pos, q_quat=q_quat, q_vel=q_vel, q_ang=q_ang)

def main():
    parser = argparse.ArgumentParser(description="Plot optimized states/inputs for Mujoco (payload+quads or quads-only).")
    parser.add_argument("--result", required=True, help="YAML with 'states' and 'actions'.")
    parser.add_argument("--model", required=True, help="Model YAML with num_robots (optional), limits, dt, etc.")
    parser.add_argument("--outdir", default="out", help="Output parent directory; figures go to outdir/plots/")
    args = parser.parse_args()

    result = load_yaml(args.result)
    model  = load_yaml(args.model)

    if "states" not in result:
        raise ValueError("Result file missing 'states'.")
    actions = result.get("actions", None)
    states  = np.asarray(result["states"], float)

    num_robots_hint = model.get("num_robots", None)
    parsed = parse_states(states, num_robots_hint=num_robots_hint)

    dt      = float(model.get("dt", 0.05))
    max_vel = float(model.get("max_vel", 0.0))
    max_w   = float(model.get("max_angular_vel", 0.0))
    max_f   = float(model.get("max_f", 0.0))
    l0 = 0.5  # only used if payload exists

    N = states.shape[0]
    t = np.arange(N) * dt

    plots_dir = ensure_outdir(args.outdir)
    pdf_path = plots_dir / "summary.pdf"
    pdf = PdfPages(pdf_path)

    dashed_v = (-max_vel, max_vel) if max_vel > 0 else None
    dashed_w = (-max_w, max_w) if max_w > 0 else None

    # Payload plots (if present)
    if parsed["has_payload"]:
        save_xyz_stack(t,
                       [parsed["p_pos"][:,0], parsed["p_pos"][:,1], parsed["p_pos"][:,2]],
                       "Payload position",
                       "m",
                       plots_dir / "payload_position.png",
                       dashed_limits=None, pdf=pdf)

        save_xyz_stack(t,
                       [parsed["p_vel"][:,0], parsed["p_vel"][:,1], parsed["p_vel"][:,2]],
                       "Payload linear velocity",
                       "m/s",
                       plots_dir / "payload_velocity.png",
                       dashed_limits=dashed_v, pdf=pdf)

        save_xyz_stack(t,
                       [parsed["p_ang"][:,0], parsed["p_ang"][:,1], parsed["p_ang"][:,2]],
                       "Payload angular velocity",
                       "rad/s",
                       plots_dir / "payload_angular_velocity.png",
                       dashed_limits=dashed_w, pdf=pdf)

    # Quads plots
    R = parsed["R"]
    for r in range(R):
        qpos = parsed["q_pos"][r]
        qvel = parsed["q_vel"][r]
        qang = parsed["q_ang"][r]

        save_xyz_stack(
            t, [qpos[:,0], qpos[:,1], qpos[:,2]],
            f"Quad {r+1} position", "m",
            plots_dir / f"quad{r+1}_position.png",
            dashed_limits=None, pdf=pdf
        )
        save_xyz_stack(
            t, [qvel[:,0], qvel[:,1], qvel[:,2]],
            f"Quad {r+1} linear velocity", "m/s",
            plots_dir / f"quad{r+1}_velocity.png",
            dashed_limits=dashed_v, pdf=pdf
        )
        save_xyz_stack(
            t, [qang[:,0], qang[:,1], qang[:,2]],
            f"Quad {r+1} angular velocity", "rad/s",
            plots_dir / f"quad{r+1}_angular_velocity.png",
            dashed_limits=dashed_w, pdf=pdf
        )

    # Inputs per robot
    if actions is not None and len(actions) > 0:
        U = np.asarray(actions, dtype=float)
        ta = t[:U.shape[0]]
        # try to split evenly per robot:
        R = parsed["R"]
        per_robot = U.shape[1] // max(R,1)
        if per_robot * R != U.shape[1]:
            # fall back: plot a single inputs figure
            save_inputs(ta, U, "Inputs (all channels)", plots_dir / "inputs_all.png", max_f=max_f, pdf=pdf)
        else:
            for r in range(R):
                ur = U[:, r*per_robot:(r+1)*per_robot]
                save_inputs(ta, ur, f"Inputs robot {r+1}",
                            plots_dir / f"inputs_robot{r+1}.png", max_f=max_f, pdf=pdf)

    # Cable slackness only if payload exists
    if parsed["has_payload"]:
        ppos = parsed["p_pos"]
        for r in range(parsed["R"]):
            qpos = parsed["q_pos"][r]
            l = np.linalg.norm(qpos - ppos, axis=1)
            slack = l - l0
            save_single_series(
                t, slack, f"slack (l - {l0})",
                f"Cable slackness robot {r+1}",
                "meters",
                plots_dir / f"cable_slack_robot{r+1}.png",
                ylimits=(-0.05, 0.05),
                pdf=pdf
            )

    pdf.close()
    print(f"Saved figures to: {plots_dir} and {pdf_path}")

if __name__ == "__main__":
    main()
