import subprocess
from pathlib import Path
import argparse
import yaml

def gen_pdf(output_path):
    """Generate PDF from LaTeX."""
    subprocess.run(['pdflatex', output_path.with_suffix(".tex")], check=True, cwd=output_path.parent)
    # Delete temporary files
    output_path.with_suffix(".aux").unlink()
    output_path.with_suffix(".log").unlink()

parser = argparse.ArgumentParser(description="Process environment statistics.")
parser.add_argument("inp", help="inpput_yaml.")
args = parser.parse_args()
inp = args.inp
output_file = f"final_table_unicycles_{inp}.tex"

from jinja2 import Template

# Environments data
environments = {
    "Wall": ["Wall, 2 robots", "Wall, 3 robots", "Wall, 4 robots", "Wall, 5 robots", "Wall, 6 robots"],
}

# Methods and robot types
methods = ["Ours", "BL"]  # Ours and Baseline (BL)
robot_types = ["UR"]  # UR: Unicycles with Rods, MP: Multirotors with Cables

def create_table(data, output_file):
    """Generates a LaTeX table from the given data."""
    # LaTeX template
    template = Template(r"""
\documentclass[twocolumn]{article}
\usepackage{booktabs}
\usepackage{xcolor}
\usepackage{multirow}
\usepackage{siunitx}

\begin{document}

% Compact table settings
\renewcommand{\arraystretch}{1.02} % Further tighten row height
\setlength{\tabcolsep}{5pt}       % Further tighten column padding
\begin{table}[t]
\caption{Simulation Results.
Shown are mean values for the success rate, cost and computational time over 10 runs for unicycles with rods (UR) with a timelimit of \SI{350}{s}. Standard deviation is small gray. Percentages are success rates. F: failed.}
\centering
\footnotesize
\begin{tabular}{|c||c|c||c|c||c|c|}
\hline
\multirow{3}{*}{\textbf{Environment, robots}} 
& \multicolumn{2}{c||}{\textbf{Success [\%]} $\uparrow$}  
& \multicolumn{2}{c||}{\textbf{Cost} [s] $\downarrow$}
& \multicolumn{2}{c|}{\textbf{Time} [s] $\downarrow$} \\
\cline{2-7}
& \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} 
& \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} 
& \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} \\
\cline{2-7}
\hline
{% for group, envs in environments.items() %}
{% for env in envs %}
{{ env }}
{% for metric in ['success', 'cost', 'time'] %}
{% for robot in robot_types %}
{% for method in methods %}
& {% if robot in data.get(env, {}).get(method, {}) and metric in data[env][method][robot] %}
\scriptsize
{% if metric == 'success' %}
{% set ours = data[env]["Ours"][robot][metric] %}
{% set bl = data[env]["BL"][robot][metric] %}
{% if ours is not none and bl is not none %}
    {% if method == 'Ours' and ours > bl %}
    \textbf{{ "{{ %.1f }}" | format(ours) }}
    {% elif method == 'BL' and bl > ours %}
    \textbf{{ "{{ %.1f }}" | format(bl) }}
    {% elif ours == bl %}
    \textbf{{ "{{ %.1f }}" | format(data[env][method][robot][metric]) }}
    {% else %}
    {{ "%.1f" | format(data[env][method][robot][metric]) }}
    {% endif %}
{% elif ours is not none %}
    {% if method == 'Ours' %}
    \textbf{{ "{{ %.1f }}" | format(ours) }}
    {% else %}
    F
    {% endif %}
{% elif bl is not none %}
    {% if method == 'BL' %}
    \textbf{{ "{{ %.1f }}" | format(bl) }}
    {% else %}
    F
    {% endif %}
{% else %}
-
{% endif %}
{% else %}
{% set ours = data[env]["Ours"][robot][metric][0] if data[env]["Ours"][robot][metric] is not none else None %}
{% set bl = data[env]["BL"][robot][metric][0] if data[env]["BL"][robot][metric] is not none else None %}
{% if ours is not none and bl is not none %}
    {% if method == 'Ours' and ours < bl %}
    {\textbf{{ "{{" ~ "%.1f" | format(ours) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["Ours"][robot][metric][1]) }}}}
    {% elif method == 'BL' and bl < ours %}
    {\textbf{{ "{{" ~ "%.1f" | format(bl) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["BL"][robot][metric][1]) }}}}
    {% else %}
    {{ "%.1f" | format(data[env][method][robot][metric][0]) }} {\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env][method][robot][metric][1]) }}}
    {% endif %}
{% elif ours is not none %}
    {% if method == 'Ours' %}
    {\textbf{{ "{{" ~ "%.1f" | format(ours) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["Ours"][robot][metric][1]) }}}}
    {% else %}
    F
    {% endif %}
{% elif bl is not none %}
    {% if method == 'BL' %}
    {\textbf{{ "{{" ~ "%.1f" | format(bl) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["BL"][robot][metric][1]) }}}}
    {% else %}
    F
    {% endif %}
{% else %}
F
{% endif %}
{% endif %}
{% else %} - {% endif %}
{% endfor %}
{% endfor %}
{% endfor %}
\\
{% endfor %}
{% if not loop.last %}
\hline
{% endif %}
{% endfor %}
\hline
\end{tabular}
\label{table2}
\end{table}
\end{document}
""")
    # Render the LaTeX table
    latex_table = template.render(data=data, environments=environments, methods=methods, robot_types=robot_types).strip() 

    # Save the LaTeX table to a file without extra lines
    with open(output_file, "w") as f:
        f.write("\n".join([line.strip() for line in latex_table.splitlines() if line.strip()]))

if __name__ == "__main__":


    with open(f"../final_data_{inp}.yaml", 'r') as f:
        data = yaml.safe_load(f)

    create_table(data, output_file)

    gen_pdf(Path(output_file))
