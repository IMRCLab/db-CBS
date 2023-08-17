

import unittest



import subprocess



class TestStandAlone(unittest.TestCase):






    def test_opti1(self):
        build_cmd = [
            "make", "multirobot_optimization"]

        run_cmd_new = [
            "./multirobot_optimization",
            "--env",
            "../example/straight.yaml",
            "--init",
            "../example/guess_indiv_straight.yaml",
            "--out",
            "buu.yaml",
            "--s",
            "1",
            ">",
            "/tmp/db_log.txt"]



        visualize_cmd = [
            "python3",
            "../scripts/visualize.py",
            "../example/straight.yaml",
            "--result",
            "/tmp/check5.yaml",
            "--video",
            "straight_solution_optimized.mp4"]



        for i in [build_cmd, run_cmd_new, visualize_cmd]:

            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0


    def test_opti2(self):
        run_cmd_old = [
            "./multirobot_optimization",
            "--env",
            "../example/straight.yaml",
            "--init",
            "../example/guess_indiv_straight.yaml",
            "--out",
            "buu.yaml",
            "--s",
            "0",
            ">",
            "/tmp/db_log.txt"]

        build_cmd = [
            "make", "multirobot_optimization"]

        visualize_cmd = [
            "python3",
            "../scripts/visualize.py",
            "../example/straight.yaml",
            "--result",
            "/tmp/check5.yaml",
            "--video",
            "straight_solution_optimized.mp4"]


        print("old optimization")
        for i in [build_cmd, run_cmd_old, visualize_cmd]:
            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0

    def test_cbs(self):
        build_cmd = [ "make" , "db_cbs" ]
        cmd_db_cbs = ["./db_cbs", "-i", "../example/classic.yaml", "-m", "../results/dbg/motions.msgpack", "-o", "classic_debug.yaml", "--jnt", "classic_debug_joint.yaml", "--opt", "classic_debug_opt.yaml" ]
        cmd_vis = [ "python3", "../scripts/visualize.py", "../example/classic.yaml", "--result", "classic_debug_opt.yaml",  "--video", "bb.mp4" ]
        for i in [build_cmd, cmd_db_cbs, cmd_vis]:
            print("running cmd")
            print(' '.join(i))

            out = subprocess.run(i)
            assert out.returncode == 0



if __name__ == "__main__":
    unittest.main()



