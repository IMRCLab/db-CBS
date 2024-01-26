import matplotlib.pyplot as plt
import yaml
import matplotlib.pyplot as plt
import msgpack
import yaml
import subprocess 
import shutil
import pathlib

def plot_search_tree(ax, trajs, sol):
    starts = []
    N = len(trajs)
    for i in range(N):
        states = trajs[i]["states"]
        X = [s[0] for s in states]
        Y = [s[1] for s in states]
        starts.append([states[0][0], states[0][1]])
        ax.plot(X, Y, color=".5", alpha=0.2)
        # print("saving ", i)
    # plt.savefig("/home/akmarak-laptop/plot.png")

    x_sol = [X[0] for X in sol["states"]]
    y_sol = [X[1] for X in sol["states"]]
    ax.plot(x_sol, y_sol, color="orange")
    ax.scatter([s[0] for s in starts], [s[1] for s in starts], s=2, color="k")


def plot_motions(ax, trajs, sol, video=True):
    starts = []
    N = len(trajs)
    fps, duration = 24, 100
    for i in range(0,10,N):
        states = trajs[i]["states"]
        X = [s[0] for s in states]
        Y = [s[1] for s in states]
        starts.append([states[0][0], states[0][1]])
        ax.plot(X, Y, color=".5", alpha=0.2)
        print("saving ", i)
        plt.savefig(f"../dynoplan/plot/fig_{i}.png")

    x_sol = [X[0] for X in sol["states"]]
    y_sol = [X[1] for X in sol["states"]]
    ax.plot(x_sol, y_sol, color="orange")
    ax.scatter([s[0] for s in starts], [s[1] for s in starts], s=2, color="k")
    plt.savefig(f"../dynoplan/plot/fig_{i+1}.png")
    if(video):
        subprocess.call(["ffmpeg","-y","-r",str(fps),"-i", "../dynoplan/plot/fig_%d.png","-vcodec","mpeg4", "-qscale","5", "-r", str(fps), "video.mp4"])

def plot_motions_no_solution(ax, trajs, video=True):
    starts = []
    N = len(trajs)
    fps, duration = 24, 100
    folder = f"../dynoplan/plot/"
    pathlib.Path(folder).mkdir(parents=True,exist_ok=True)
    for i in range(0,N,10):
        states = trajs[i]["states"]
        X = [s[0] for s in states]
        Y = [s[1] for s in states]
        starts.append([states[0][0], states[0][1]])
        ax.plot(X, Y, color=".5", alpha=0.2)
        print("saving ", i)
        file = f"{folder}/fig_{i}.png"
        plt.savefig(file)

    ax.scatter([s[0] for s in starts], [s[1] for s in starts], s=2, color="k")
    plt.savefig(f"../dynoplan/plot/fig_{i+1}.png")
    if(video):
        subprocess.call(["ffmpeg","-y","-r",str(fps),"-i", "../dynoplan/plot/fig_%d.png","-vcodec","mpeg4", "-qscale","5", "-r", str(fps), "video.mp4"])

# # read tmp_trajs
filename_motions = "../dynoplan/expanded_trajs.yaml"
with open(filename_motions) as motions_file:
    motions = yaml.safe_load(motions_file)
tmp_trajs = motions["trajs"]
print(len(tmp_trajs))
# read msgpacks
# msg_file = "/home/akmarak-laptop/IMRC/db-CBS/dynoplan/data/motion_primitives/unicycle1_v0/unicycle1_v0.bin.less.bin.msgpack"
# with open(msg_file, "rb") as f:
#     data = msgpack.unpackb(f.read(), raw=False)
# print(len(data["data"]))
# N = len(data["data"])
# trajs = data["data"]

# # solution
# result_file = "/home/akmarak-laptop/IMRC/db-CBS/results/swap3_trailer_09/db-cbs/000/result_dbcbs.yaml"
# with open(result_file, "r") as f:
#     data_sol = yaml.safe_load(f)
# result = data_sol["result"][1]

# x_sol = [X[0] for X in result["states"]]
# y_sol = [X[1] for X in result["states"]]

fig = plt.figure()
ax = fig.add_subplot(111, aspect="equal")
ax.set_xlim([0, 6])
ax.set_ylim([0, 6])

plt.tick_params(
    top=False, bottom=False, left=False, right=False, labelleft=False, labelbottom=False
)
plt.axis("off")
fig.tight_layout()

plot_motions_no_solution(ax, tmp_trajs)
# plot_motions(ax, tmp_trajs, result)
# plot_search_tree(ax, trajs, result)
# ax.set_aspect("equal", adjustable="box")
# plt.show()
