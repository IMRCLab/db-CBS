import matplotlib.pyplot as plt
import yaml
import subprocess 
import pathlib
import argparse

def plot_expanded_trajs(filename_env, filename_trajs, filename_result = None, filename_video = None):
    with open(filename_env) as env_file:
      env = yaml.safe_load(env_file)
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect="equal")
    ax.set_xlim(env["environment"]["min"][0], env["environment"]["max"][0])
    ax.set_ylim(env["environment"]["min"][1], env["environment"]["max"][1])

    plt.tick_params(
    top=False, bottom=False, left=False, right=False, labelleft=False, labelbottom=False)
    plt.axis("off")
    fig.tight_layout()
    # read trajs
    with open(filename_trajs) as motions_file:
        motions = yaml.safe_load(motions_file)
    trajs = motions["trajs"]
    starts = []
    N = len(trajs)
    fps, duration = 24, 100
    folder = f"../dynoplan/plot/" # maybe better way with not saving images
    pathlib.Path(folder).mkdir(parents=True,exist_ok=True)
    for i in range(0,N):
        states = trajs[i]["states"]
        X = [s[0] for s in states]
        Y = [s[1] for s in states]
        starts.append([states[0][0], states[0][1]])
        ax.plot(X, Y, color=".5", alpha=0.2)
        print("saving ", i)
        file = f"{folder}/fig_{i}.png"
        plt.savefig(file)
    if filename_result is not None:
        with open(filename_result, "r") as f:
            data_sol = yaml.safe_load(f)
        result = data_sol["result"][1]
        x_sol = [X[0] for X in result["states"]]
        y_sol = [X[1] for X in result["states"]]
        ax.plot(x_sol, y_sol, color="orange")
    ax.scatter([s[0] for s in starts], [s[1] for s in starts], s=2, color="k")
    plt.savefig(f"../dynoplan/plot/fig_{i+1}.png")
    if filename_video is not None:
        subprocess.call(["ffmpeg","-y","-r",str(fps),"-i", "../dynoplan/plot/fig_%d.png","-vcodec","mpeg4", "-qscale","5", "-r", str(fps), filename_video])


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--trajs", help="file with expanded trajectories during the search")
  parser.add_argument("--result", help="file with the final solution")
  parser.add_argument("--video", help="output file for video")
  args = parser.parse_args()

  plot_expanded_trajs(args.env, args.trajs, args.video)

if __name__ == "__main__":
  main()