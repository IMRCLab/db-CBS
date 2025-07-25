import subprocess
from pathlib import Path


def gen_pdf(output_path):
    """Generate PDF from LaTeX."""
    subprocess.run(['pdflatex', output_path.with_suffix(".tex")], check=True, cwd=output_path.parent)
    # Delete temporary files
    output_path.with_suffix(".aux").unlink()
    output_path.with_suffix(".log").unlink()


output_file = "final_table.tex"

from jinja2 import Template

# Environments data (reordered: Window, Wall, Forest)
environments = {
    "Window": ["Window 2", "Window 3"],
    "Forest": ["Forest 2", "Forest 3"],
    "lego": ["lego 2", "lego 3"],
}

# Methods and robot types
methods = ["Ours", "BL"]  # Ours and Baseline (BL)
robot_types = ["UR", "MP"]  # UR: Unicycles with Rods, MP: Multirotors with Cables

def create_table(data, output_file):
    """Generates a LaTeX table from the given data."""
    # LaTeX template
    template = Template(r"""
\documentclass[twocolumn]{article}
\usepackage{booktabs}
\usepackage{xcolor}
\usepackage{multirow}

\begin{document}

% Compact table settings
\renewcommand{\arraystretch}{1.0} % Further tighten row height
\setlength{\tabcolsep}{5pt}       % Further tighten column padding

\begin{table*}[h!]
\caption{Performance Metrics for Different Methods Across Environments, Robot Types, and Methods.}
\centering
\footnotesize
\begin{tabular}{|c|c|c|c|c|c|c|c|c|c|c|c|c|}
\hline
\multirow{3}{*}{\textbf{Environment}} 
& \multicolumn{4}{c|}{\textbf{Energy} [Wh]} 
& \multicolumn{4}{c|}{\textbf{Error } [m]} 
& \multicolumn{4}{c|}{\textbf{Time} [s]} \\
\cline{2-13}
& \multicolumn{2}{c|}{\textbf{UR}} & \multicolumn{2}{c|}{\textbf{MP}} 
& \multicolumn{2}{c|}{\textbf{UR}} & \multicolumn{2}{c|}{\textbf{MP}} 
& \multicolumn{2}{c|}{\textbf{UR}} & \multicolumn{2}{c|}{\textbf{MP}} \\
\cline{2-13}
& \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} & \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} 
& \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} & \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} 
& \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} & \scriptsize \textbf{Ours} & \scriptsize \textbf{BL} \\
\hline
{% for group, envs in environments.items() %}
{% for env in envs %}
{{ env }}
{% for metric in ['energy', 'error', 'time'] %}
{% for robot in robot_types %}
{% for method in methods %}
& {% if robot in data.get(env, {}).get(method, {}) and metric in data[env][method][robot] %}
\scriptsize
{% if metric == 'time' %}
{% set ours = data[env]["Ours"][robot][metric] %}
{% set bl = data[env]["BL"][robot][metric] %}
{% if ours is not none and bl is not none %}
    {% if method == 'Ours' and ours < bl %}
    \textbf{{ "{{" ~ ours ~ "}}" }}
    {% elif method == 'BL' and bl < ours %}
    \textbf{{ "{{" ~ bl ~ "}}" }}
    {% else %}
    {{ data[env][method][robot][metric] }}
    {% endif %}
{% elif ours is not none %}
    {% if method == 'Ours' %}
    \textbf{{ "{{" ~ ours ~ "}}" }}
    {% else %}
    -
    {% endif %}
{% elif bl is not none %}
    {% if method == 'BL' %}
    \textbf{{ "{{" ~ bl ~ "}}" }}
    {% else %}
    -
    {% endif %}
{% else %}
-
{% endif %}
{% else %}
{% set ours = data[env]["Ours"][robot][metric][0] if data[env]["Ours"][robot][metric] is not none else None %}
{% set bl = data[env]["BL"][robot][metric][0] if data[env]["BL"][robot][metric] is not none else None %}
{% if ours is not none and bl is not none %}
    {% if method == 'Ours' and ours < bl %}
    {\textbf{{ "{{" ~ "%.3f" | format(ours) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["Ours"][robot][metric][1]) }}}}
    {% elif method == 'BL' and bl < ours %}
    {\textbf{{ "{{" ~ "%.3f" | format(bl) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["BL"][robot][metric][1]) }}}}
    {% else %}
    {{ "%.3f" | format(data[env][method][robot][metric][0]) }} {\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env][method][robot][metric][1]) }}}
    {% endif %}
{% elif ours is not none %}
    {% if method == 'Ours' %}
    {\textbf{{ "{{" ~ "%.3f" | format(ours) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["Ours"][robot][metric][1]) }}}}
    {% else %}
    -
    {% endif %}
{% elif bl is not none %}
    {% if method == 'BL' %}
    {\textbf{{ "{{" ~ "%.3f" | format(bl) ~ "}}" }}\hspace{0.5em}{\tiny \textcolor{gray}{{ "{%.1f}" | format(data[env]["BL"][robot][metric][1]) }}}}
    {% else %}
    -
    {% endif %}
{% else %}
-
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
\end{table*}

\end{document}
""")    # Render the LaTeX table
    latex_table = template.render(data=data, environments=environments, methods=methods, robot_types=robot_types).strip()

    # Save the LaTeX table to a file without extra lines
    with open(output_file, "w") as f:
        f.write("\n".join([line.strip() for line in latex_table.splitlines() if line.strip()]))

if __name__ == "__main__":
    # Full data for all environments, methods, and robot types
    data = {}
    data["Window 2"] = {
        "Ours": {
            "UR": {
                "energy": (None, None), "error": (None, None), "time":None
            },
            "MP": {
                "energy": (0.006, 0.0), "error": (0.084, 0.05), "time": 4.3
            },
        },
        "BL": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.01, 0.0), "error": (0.047, 0.03), "time": 7.5
            },
        },
    }


    data["Window 3"] = {
        "Ours": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.007, 0.00), "error": (0.147, 0.06), "time": 4.2
            },
        },
        "BL": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.017,0.0), "error": (0.079, 0.04), "time": 8.5
            },
        },
    }


    data["Forest 2"] = {
        "Ours": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.007, 0.00), "error": (0.061, 0.03), "time": 5.0
            },
        },
        "BL": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.01,0), "error": (0.033, 0.02), "time": 8.2
            },
        },
    }


    data["Forest 3"] = {
        "Ours": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.009, 0.00), "error": (0.118, 0.06), "time": 4.5
            },
        },
        "BL": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (0.011,0), "error": (0.07, 0.05), "time": 7.7
            },
        },
    }


    data["lego 2"] = {
        "Ours": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (None, None), "error": (None, None), "time": None
            },
        },
        "BL": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (None,None), "error": (None, None), "time": None
            },
        },
    }


    data["lego 3"] = {
        "Ours": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (None, None), "error": (None, None), "time": None
            },
        },
        "BL": {
            "UR": {
                "energy": (None,None), "error": (None, None), "time": None
            },
            "MP": {
                "energy": (None,None), "error": (None, None), "time": None
            },
        },
    }





    # for env_group, env_list in environments.items():
    #     for env in env_list:

    # Call the create_table function
    create_table(data, output_file)

    # Generate PDF
    gen_pdf(Path(output_file))
