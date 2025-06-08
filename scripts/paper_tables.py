from pathlib import Path
import benchmark_table
from benchmark_table import write_table
import os
import shutil

def write_table1(trials, timelimit):
  instances = [
    "swap2_unicycle_sphere",
    "alcove_unicycle_sphere",
    "at_goal_unicycle_sphere",
    "window4_unicycle_sphere",
  ]
  algs = [
    "sst",
    "s2m2",
    "k-cbs",
    "db-cbs",
  ]

  write_table(instances, algs, Path("../results"), "paper_table1.pdf", trials, timelimit)

def write_table2(trials, timelimit):
  instances = [
    "gen_p10_n2_*_unicycle_sphere",
    "gen_p10_n4_*_unicycle_sphere",
    "gen_p10_n8_*_unicycle_sphere",
  ]
  algs = [
    "sst",
    "s2m2",
    "k-cbs",
    "db-cbs",
  ]

  write_table(instances, algs, Path("../results"), "paper_table2.pdf", 10, timelimit, True)

def write_table3(trials, timelimit):
  instances = [
    "gen_p10_n2_*_hetero",
    "gen_p10_n4_*_hetero",
    "gen_p10_n8_*_hetero",
  ]
  algs = [
    "sst",
    "k-cbs",
    "db-cbs",
  ]

  write_table(instances, algs, Path("../results"), "paper_table3.pdf", 10, timelimit, True)

def write_table4(trials, timelimit):
  instances = [
    "swap1_unicycle",
    "swap1_double_integrator",
    "swap1_trailer",
    "swap1_unicycle2",

    "swap2_unicycle",
    "swap2_double_integrator",
    "swap2_trailer",
    "swap2_unicycle2",

    "swap3_unicycle",
    "swap3_double_integrator",
    "swap3_trailer",
    "swap3_unicycle2",

    "swap4_unicycle",
    "swap4_double_integrator",
    "swap4_trailer",
    "swap4_unicycle2",
  ]
  algs = [
    "sst",
    "k-cbs",
    "db-cbs",
  ]

  robots = [1,2,3,4]
  dynamics = ["unicycle", "double_integrator", "trailer", "unicycle2"]

  r = benchmark_table.compute_results(instances, algs, Path("../results"), trials, timelimit)
  print(r)

  output_path = Path("../results/paper_table4.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    alg_names = {
      "sst": "$\star$",
      "k-cbs": "$\dagger$",
      "db-cbs": "$\ddagger$",
    }

    dyn_names = {
      "unicycle": "unicycle $1^{\mathrm{st}}$",
      "double_integrator": "double int.",
      "trailer": "car with trailer",
      "unicycle2": "unicycle $2^{\mathrm{nd}}$",
    }

    out = r"\begin{tabular}{c "
    for d in dynamics:
      out += r" || r|r|r"
    out += "}\n"
    f.write(out)
    out = r"N "
    for k, d in enumerate(dynamics):
      if k == len(dynamics) - 1:
        out += r" & \multicolumn{3}{c}{"
      else:
        out += r" & \multicolumn{3}{c||}{"
      out += dyn_names[d]
      out += r"}"
    out += r"\\"
    f.write(out)

    out = ""
    for _ in dynamics:
      for k, alg in enumerate(algs):
        out += r"& "
        out += alg_names[alg]
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    for n in robots:
      out = ""
      out += r"\hline"
      out += str(n)
      for d in dynamics:
        for alg in algs:
          out = benchmark_table.print_and_highlight_best(out, 't^st_median', r["swap{}_{}".format(n, d)], alg, algs)
      out += r"\\"
      f.write(out)

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  benchmark_table.gen_pdf(output_path)

def write_table5(trials, timelimit):
  instances = [
    "swap2_unicycle_sphere",
    "alcove_unicycle_sphere",
    "at_goal_unicycle_sphere",
    # "window4_unicycle_sphere",

    "<<HLINE>>",

    "gen_p10_n2_*_unicycle_sphere",
    "gen_p10_n4_*_unicycle_sphere",
    "gen_p10_n8_*_unicycle_sphere",

    "<<HLINE>>",

    "gen_p10_n2_*_hetero",
    "gen_p10_n4_*_hetero",
    "gen_p10_n8_*_hetero",
  ]
  trials = [trials]*4 + [10*trials]*7
  algs = [
    "sst",
    "s2m2",
    "k-cbs",
    "db-cbs",
    "db-ecbs",
  ]

  instance_names = {
    'swap2_unicycle_sphere': "swap",
    'alcove_unicycle_sphere': "alcove",
    'at_goal_unicycle_sphere': "at goal",
    'window4_unicycle_sphere': "window4",
    'gen_p10_n2_*_unicycle_sphere': "rand (N=2)",
    'gen_p10_n4_*_unicycle_sphere': "rand (N=4)",
    'gen_p10_n8_*_unicycle_sphere': "rand (N=8)",
    'gen_p10_n2_*_hetero': "rand hetero (N=2)",
    'gen_p10_n4_*_hetero': "rand hetero (N=4)",
    'gen_p10_n8_*_hetero': "rand hetero (N=8)",
  }

  alg_names = {
    "sst": "SST*",
    "s2m2": "S2M2",
    "k-cbs": "k-CBS",
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
  }

  result = benchmark_table.compute_results(instances, algs, Path("../results"), trials, timelimit, True)
  output_path = Path("../results/paper_table5.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    out = r"\begin{tabular}{c || c"
    for alg in algs:
      out += r" || r|r|r" # |r
    out += "}\n"
    f.write(out)
    out = r"\# & Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        out += r" & \multicolumn{3}{c}{" # 4
      else:
        out += r" & \multicolumn{3}{c||}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    out = r"& "
    for alg in algs:
      # out += r" & $p$ & $t [s]$ & $J [s]$ & $r [\%]$"
      out += r" & $p$ & $t [s]$ & $J [s]$" # without notion of regret
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    r_number = 0
    for instance in instances:

      if instance == "<<HLINE>>":
        f.write(r"\hline")
        f.write("\n")
        continue

      out = ""
      out += r"\hline"
      out += "\n"
      out += "{} & ".format(r_number+1)
      if instance in instance_names:
        out += instance_names[instance]
      else:
        out += "{} ".format(instance.replace("_", "\_"))

      for alg in algs:

        out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 't^st_median', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 'J^st_median', result[instance], alg, algs)
        # out = benchmark_table.print_and_highlight_best(out, 'Jr^st_median', result[instance], alg, algs, digits=0) // without notion of regret

      out += r"\\"
      f.write(out)
      r_number += 1

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  benchmark_table.gen_pdf(output_path)

def write_table6(trials, timelimit):
  regret = False
  instances = [
    "drone2c",
    # "drone4c",
    # "drone8c",
    # "drone10c",
    # "drone12c",
    # "drone16c",
    # "wall_drone8c",
    # "wall_drone10c",
  ]
  instance_names = {
    'drone2c': "wall2",
    # 'drone4c': "wall4",
    # 'drone8c': "wall8",
    # 'drone10c': "wall10",
    # 'drone12c': "wall12",
    # 'drone16c': "wall16",
    # 'wall_drone8c': "window8",
    # 'wall_drone10c': "window10",
  }
  algs = [
    "tro-18",
    "db-ecbs-conservative",
    "db-ecbs-residual",
    
  ]
  # map to a shorter name for the table
  alg_names = {
    "tro-18": "MAPF/C+POST",
    "db-ecbs-conservative": "db-ECBS-C",
    "db-ecbs-residual": "db-ECBS-R",
  }
  # result = benchmark_table.compute_results(instances, algs, Path("../results"), trials, timelimit, True)
  result = benchmark_table.compute_results_with_std(instances, algs, Path("../../../Desktop/"), trials, timelimit, True)
  
  # manually enter results for tro-18
  result_d2 = result["drone2c"]
  result_d2["tro-18"] = {
    'success': 1.0,
    't^st_mean': 0.983,
    'tr^st_mean': None,
    'J^st_mean': 60.7,
    'Jr^st_mean': None,
    'J^f_mean': 60.7,
    'Jr^f_mean': None,
  }
  n = 4
  result_d4 = result["drone4c"]
  result_d4["tro-18"] = {
    'success': 1.0,
    't^st_mean': 1.643,
    'tr^st_mean': None,
    'J^st_mean': 147.3,
    'Jr^st_mean': None,
    'J^f_mean': 147.3,
    'Jr^f_mean': None,
  }
  # n = 8
  result_d8 = result["drone8c"]
  result_d8["tro-18"] = {
    'success': 1.0,
    't^st_mean': 3.187,
    'tr^st_mean': None,
    'J^st_mean': 297.11,
    'Jr^st_mean': None,
    'J^f_mean': 297.11,
    'Jr^f_mean': None,
  }
  # n = 10
  result_d10 = result["drone10c"]
  result_d10["tro-18"] = {
    'success': 1.0,
    't^st_mean': 3.969,
    'tr^st_mean': None,
    'J^st_mean': 373,
    'Jr^st_mean': None,
    'J^f_mean': 373,
    'Jr^f_mean': None,
  }
  # n = 12
  result_d12 = result["drone12c"]
  result_d12["tro-18"] = {
    'success': 1.0,
    't^st_mean': 4.894,
    'tr^st_mean': None,
    'J^st_mean': 439.8,
    'Jr^st_mean': None,
    'J^f_mean': 439.8,
    'Jr^f_mean': None,
  }
  # n = 16
  result_d16 = result["drone16c"]
  result_d16["tro-18"] = {
    'success': 1.0,
    't^st_mean': 7.727,
    'tr^st_mean': None,
    'J^st_mean': 657.4,
    'Jr^st_mean': None,
    'J^f_mean': 657.4,
    'Jr^f_mean': None,
  }
  # wall8
  result_wall8 = result["wall_drone8c"]
  result_wall8["tro-18"] = {
    'success': '*',
    't^st_mean': '*',
    'tr^st_mean': '*',
    'J^st_mean': '*',
    'Jr^st_mean': '*',
    'J^f_mean': '*',
    'Jr^f_mean': '*',
  }
  # wall10
  result_wall10 = result["wall_drone10c"]
  result_wall10["tro-18"] = {
    'success': '*',
    't^st_mean': '*',
    'tr^st_mean': '*',
    'J^st_mean': '*',
    'Jr^st_mean': '*',
    'J^f_mean': '*',
    'Jr^f_mean': '*',
  }

  output_path = Path("../results/paper_table_std.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    out = r"\begin{tabular}{c | c"
    for alg in algs:
      if not regret:
        out += r" | r|r|r|r"
      else:
        out += r" | r|r|r"
    out += "}\n"
    f.write(out)
    out = r"\# & Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if not regret:
          out += r" & \multicolumn{4}{c}{"
        else:
          out += r" & \multicolumn{3}{c}{"
      else:
        if not regret:
          out += r" & \multicolumn{4}{c|}{"
        else:
          out += r" & \multicolumn{3}{c|}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    out = r"& "
    if not regret:
      for alg in algs:
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
    else:
      for alg in algs:
        out += r" & $p$ & $t_r^{\mathrm{st}} [\%]$ & $J_r^{f} [\%]$"
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    for r_number, row in enumerate(instances): 

      out = ""
      out += r"\hline"
      out += "\n"
      out += "{} & ".format(r_number+1)
      if row in instance_names:
        out += instance_names[row]
      else:
        out += "{} ".format(row.replace("_", "\_"))

      for alg in algs:
        if not regret:
          out = benchmark_table.print_and_highlight_best_max(out, 'success', result[row], alg, algs)
          out = benchmark_table.print_and_highlight_best(out, 't^st_median', result[row], alg, algs)
          out = benchmark_table.print_and_highlight_best(out, 'J^st_median', result[row], alg, algs)
          out = benchmark_table.print_and_highlight_best(out, 'J^f_median', result[row], alg, algs)
        else:
          out = benchmark_table.print_and_highlight_best_max(out, 'success', result[row], alg, algs)
          out = benchmark_table.print_and_highlight_best(out, 'tr^st_median', result[row], alg, algs)
          out = benchmark_table.print_and_highlight_best(out, 'Jr^f_median', result[row], alg, algs)

      out += r"\\"
      f.write(out)

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  # run pdflatex
  benchmark_table.gen_pdf(output_path)
  

# table for tro. It has the notion of regret w.r.t db-ecbs, and skips it in the table since it's always 0
def write_table7(trials, timelimit):
  instances = [
    "swap2_unicycle_sphere",
    "alcove_unicycle_sphere",
    "at_goal_unicycle_sphere",

    "<<HLINE>>",

    "gen_p10_n2_*_unicycle_sphere",
    "gen_p10_n4_*_unicycle_sphere",
    "gen_p10_n8_*_unicycle_sphere",

    "<<HLINE>>",

    "gen_p10_n2_*_hetero",
    "gen_p10_n4_*_hetero",
    "gen_p10_n8_*_hetero",
  ]
  trials = [trials]*4 + [10*trials]*7
  algs = [
    "sst",
    "s2m2",
    "k-cbs",
    "db-cbs",
    "db-ecbs",
  ]

  instance_names = {
    'swap2_unicycle_sphere': "swap",
    'alcove_unicycle_sphere': "alcove",
    'at_goal_unicycle_sphere': "at goal",
    'window4_unicycle_sphere': "window4",
    'gen_p10_n2_*_unicycle_sphere': "rand (N=2)",
    'gen_p10_n4_*_unicycle_sphere': "rand (N=4)",
    'gen_p10_n8_*_unicycle_sphere': "rand (N=8)",
    'gen_p10_n2_*_hetero': "rand het (N=2)",
    'gen_p10_n4_*_hetero': "rand het (N=4)",
    'gen_p10_n8_*_hetero': "rand het (N=8)",
  }

  alg_names = {
    "sst": "SST*",
    "s2m2": "S2M2",
    "k-cbs": "k-CBS",
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
  }

  result = benchmark_table.compute_results_with_std(instances, algs, Path("../results/tro-plots/2d-10/"), trials, timelimit, True)
  output_path = Path("../results/paper_table7_std.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\usepackage{xcolor}") 
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    out = r"\begin{tabular}{l"
    for alg in algs:
      if(alg == "db-ecbs"):
        out += r" |r|r|r"
      else:
        out += r" |r|r|r|r"
    out += "}\n"
    f.write(out)
    out = r"Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if(alg == "db-ecbs"):
          out += r" & \multicolumn{3}{c}{" 
        else: 
          out += r" & \multicolumn{4}{c}{" 
      else:
        out += r" & \multicolumn{4}{c}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    # out = r"& "
    out = ""
    for alg in algs:
      if(alg == "db-ecbs"):
        out += r" & $p$ & $t [s]$ & $J [s] $"
      else:
        out += r" & $p$ & $t [s]$ & $J [s]$ & $r [\%]$"
    out += r"\\"
    f.write(out)
    # f.write(r"\hline")

    r_number = 0
    for instance in instances:

      if instance == "<<HLINE>>":
        # f.write(r"\hline")
        # f.write("\n")
        continue

      out = ""
      out += r"\hline"
      out += "\n"
      # out += "{} & ".format(r_number+1)
      if instance in instance_names:
        out += instance_names[instance]
      else:
        out += "{} ".format(instance.replace("_", "\_"))

      # for alg in algs:
        # out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
        # out = benchmark_table.print_and_highlight_best(out, 't^st_median', result[instance], alg, algs)
        # out = benchmark_table.print_and_highlight_best(out, 'J^st_median', result[instance], alg, algs)
        # if(alg != "db-ecbs"):
          # out = benchmark_table.print_and_highlight_best(out, 'Jr^st_median', result[instance], alg, algs, digits=0) 
      for alg in algs:
        out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best_std(out, 't^st_mean', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best_std(out, 'J^st_mean', result[instance], alg, algs)
        if(alg != "db-ecbs"):
          out = benchmark_table.print_and_highlight_best_std(out, 'Jr^st_mean', result[instance], alg, algs, digits=0) 

      out += r"\\"
      f.write(out)
      r_number += 1

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  benchmark_table.gen_pdf(output_path)

# wall example for residual vs. conservative case
def write_table8(trials, timelimit):
  instances = [
    "wall_drone8c",
  ]
  algs = [
    "db-ecbs-residual",
    "db-ecbs-conservative",
  ]

  write_table(instances, algs, Path("../results/"), "paper_table3.pdf", trials, timelimit)

# table 6 with std
def write_table9(trials, timelimit):
  regret = False
  instances = [
    "drone2c",
    "drone4c",
    "drone8c",
    "drone10c",
    "drone12c",
    "drone16c",
    "wall_drone8c",
    "wall_drone10c",
  ]
  instance_names = {
    'drone2c': "wall2",
    'drone4c': "wall4",
    'drone8c': "wall8",
    'drone10c': "wall10",
    'drone12c': "wall12",
    'drone16c': "wall16",
    'wall_drone8c': "window8",
    'wall_drone10c': "window10",
  }
  algs = [
    "tro-18",
    "db-ecbs-conservative",
    "db-ecbs-residual",
    
  ]
  # map to a shorter name for the table
  alg_names = {
    "tro-18": "MAPF/C+POST",
    "db-ecbs-conservative": "db-ECBS-C",
    "db-ecbs-residual": "db-ECBS-R",
  }
  result = benchmark_table.compute_results_with_std(instances, algs, Path("../results/tro-plots/results-3d-5trials/"), trials, timelimit, True)
  
  # manually enter results for tro-18
  result_d2 = result["drone2c"]
  result_d2["tro-18"] = {
    'success': 1.0,
    't^st_mean': 0.983,
    't^st_std': 0.0,
    'tr^st_mean': None,
    'J^st_mean': 60.7,
    'J^st_std': 0.0,
    'Jr^st_mean': None,
    'J^f_mean': 60.7,
    'J^f_std': 0.0,
    'Jr^f_mean': None,
  }
  n = 4
  result_d4 = result["drone4c"]
  result_d4["tro-18"] = {
    'success': 1.0,
    't^st_mean': 1.643,
    't^st_std': 0.01,
    'tr^st_mean': None,
    'J^st_mean': 147.3,
    'J^st_std': 0.0,
    'Jr^st_mean': None,
    'J^f_mean': 147.3,
    'J^f_std': 0.0,
    'Jr^f_mean': None,
  }
  # n = 8
  result_d8 = result["drone8c"]
  result_d8["tro-18"] = {
    'success': 1.0,
    't^st_mean': 3.187,
    't^st_std': 0.01,
    'tr^st_mean': None,
    'J^st_mean': 297.11,
    'J^st_std': 0.0,
    'Jr^st_mean': None,
    'J^f_mean': 297.11,
    'J^f_std': 0.0,
    'Jr^f_mean': None,
  }
  # n = 10
  result_d10 = result["drone10c"]
  result_d10["tro-18"] = {
    'success': 1.0,
    't^st_mean': 3.969,
    't^st_std': 0.02,
    'tr^st_mean': None,
    'J^st_mean': 373,
    'J^st_std': 0.0,
    'Jr^st_mean': None,
    'J^f_mean': 373,
    'J^f_std': 0.0,
    'Jr^f_mean': None,
  }
  # n = 12
  result_d12 = result["drone12c"]
  result_d12["tro-18"] = {
    'success': 1.0,
    't^st_mean': 4.894,
    't^st_std': 0.01,
    'tr^st_mean': None,
    'J^st_mean': 439.8,
    'J^st_std': 0.0,
    'Jr^st_mean': None,
    'J^f_mean': 439.8,
    'J^f_std': 0.0,
    'Jr^f_mean': None,
  }
  # n = 16
  result_d16 = result["drone16c"]
  result_d16["tro-18"] = {
    'success': 1.0,
    't^st_mean': 7.727,
    't^st_std': 0.01,
    'tr^st_mean': None,
    'J^st_mean': 657.4,
    'J^st_std': 0.0,
    'Jr^st_mean': None,
    'J^f_mean': 657.4,
    'J^f_std': 0.0,
    'Jr^f_mean': None,
  }
  # wall8
  result_wall8 = result["wall_drone8c"]
  result_wall8["tro-18"] = {
    'success': '*',
    't^st_mean': '*',
    't^st_std': '-',
    'tr^st_mean': '*',
    'J^st_mean': '*',
    'J^st_std': '-',
    'Jr^st_mean': '*',
    'J^f_mean': '*',
    'J^f_std': '-',
    'Jr^f_mean': '*',
  }
  # wall10
  result_wall10 = result["wall_drone10c"]
  result_wall10["tro-18"] = {
    'success': '*',
    't^st_mean': '*',
    't^st_std': '-',
    'tr^st_mean': '*',
    'J^st_mean': '*',
    'J^st_std': '-',
    'Jr^st_mean': '*',
    'J^f_mean': '*',
    'J^f_std': '-',
    'Jr^f_mean': '*',
  }

  output_path = Path("../results/paper_table_std.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\usepackage{xcolor}") 
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")
 
    # out = r"\begin{tabular}{c | c"
    out = r"\begin{tabular}{l"
    for alg in algs:
      if not regret:
        out += r" | r|r|r|r"
      else:
        out += r" | r|r|r"
    out += "}\n"
    f.write(out)
    # out = r"\# & Instance"
    out = r"Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if not regret:
          out += r" & \multicolumn{4}{c}{"
        else:
          out += r" & \multicolumn{3}{c}{"
      else:
        if not regret:
          out += r" & \multicolumn{4}{c|}{"
        else:
          out += r" & \multicolumn{3}{c|}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    # out = r"& "
    out = ""
    if not regret:
      for alg in algs:
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
    else:
      for alg in algs:
        out += r" & $p$ & $t_r^{\mathrm{st}} [\%]$ & $J_r^{f} [\%]$"
    out += r"\\"
    f.write(out)
    # f.write(r"\hline")

    for r_number, row in enumerate(instances): 

      out = ""
      out += r"\hline"
      out += "\n"
      # out += "{} & ".format(r_number+1) # enumeration
      if row in instance_names:
        out += instance_names[row]
      else:
        # out += "{} ".format(row.replace("_", "\_"))
        out += row.replace("_", "\_")

      for alg in algs:
        out = benchmark_table.print_and_highlight_best_max(out, 'success', result[row], alg, algs)
        out = benchmark_table.print_and_highlight_best_std(out, 't^st_mean', result[row], alg, algs, skip_std_for_time=True)
        out = benchmark_table.print_and_highlight_best_std(out, 'J^st_mean', result[row], alg, algs)
        out = benchmark_table.print_and_highlight_best_std(out, 'J^f_mean', result[row], alg, algs)
        if regret:
          out = benchmark_table.print_and_highlight_best_std(out, 'Jr^f_mean', result[row], alg, algs)

      out += r"\\"
      f.write(out)

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  # run pdflatex
  benchmark_table.gen_pdf(output_path)
if __name__ == '__main__':
  trials = 5
  timelimit = 5*60
  # write_table1(trials, timelimit)
  # write_table2(trials, timelimit)
  # write_table3(trials, timelimit)
  # write_table4(trials, timelimit)
  # write_table5(trials, timelimit)
  # write_table6(trials, timelimit) # window, wall together
  write_table7(trials, timelimit) # hast std
  # write_table8(trials, timelimit)
  # write_table9(trials, timelimit) # window, wall together. As table6, but has std






