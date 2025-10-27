from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import os
import json
import glob

def build_3d_tensor(variants, param_names, targets):
    # Kombiniere alle Setup-Parameter zu einer Achse
    setup_combinations = []
    for v in variants:
        combo = tuple(f"{k}={v['setup'][k]}" for k in v['setup'].keys())
        setup_combinations.append(combo)
    unique_combos = sorted(list(set(setup_combinations)))
    max_runs = max(len(v["executions"]) for v in variants)
    num_targets = len(targets)
    tensor = np.zeros((len(unique_combos), max_runs, num_targets))
    for v in variants:
        combo = tuple(f"{k}={v['setup'][k]}" for k in v['setup'].keys())
        idx_x = unique_combos.index(combo)
        for run_idx, exec in enumerate(v["executions"]):
            for t_idx, target in enumerate(targets):
                res = exec["targets"].get(target, {"status": "nicht getestet"})
                val = 1 if res["status"] == "bestanden" else (-1 if res["status"] == "fehlgeschlagen" else 0)
                tensor[idx_x, run_idx, t_idx] = val
    return tensor, unique_combos, targets

def plot_3d_matrix(test_name, tensor, param_values, param_names, targets_list):
    fig = plt.figure(figsize=(max(14, 2*len(param_values)), 10))  # Größere Figure
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs, cs = [], [], [], []
    for i, combo in enumerate(param_values):
        for k in range(tensor.shape[1]):
            for t_idx, target in enumerate(targets_list):
                val = tensor[i, k, t_idx]
                xs.append(i)
                ys.append(k)
                zs.append(t_idx)
                cs.append('g' if val == 1 else ('r' if val == -1 else 'gray'))
    ax.scatter(xs, ys, zs, c=cs, s=80)
    ax.set_xticks(range(len(param_values)))
    ax.set_xticklabels(['\n'.join(combo) for combo in param_values], rotation=30, ha='right', fontsize=8)
    ax.set_yticks(range(tensor.shape[1]))
    ax.set_yticklabels([f"Run {i+1}" for i in range(tensor.shape[1])], fontsize=10)
    ax.set_zticks(range(len(targets_list)))
    ax.set_zticklabels(targets_list, fontsize=10)
    ax.set_ylabel('Run')
    ax.set_zlabel('Ziel')
    ax.set_title(f"3D Testmatrix – {test_name}")
    plt.subplots_adjust(left=0.15, right=0.85, bottom=0.25, top=0.85)  # Mehr Rand
    plt.tight_layout()
    plt.show()

def load_results(results_file):
    with open(results_file, 'r') as f:
        return json.load(f)

def load_results_file():
    # Suche rekursiv nach Ergebnisdateien
    files = glob.glob("**/test_results_*.json", recursive=True)
    if not files:
        raise FileNotFoundError("Keine test_results_*.json Datei in diesem Verzeichnis oder Unterordnern gefunden!")
    print("\nVerfügbare Ergebnisdateien:")
    for i, f in enumerate(files, 1):
        print(f"{i}: {f}")
    choice = input("\nDatei wählen (Nummer): ").strip()
    idx = int(choice) - 1 if choice.isdigit() and 1 <= int(choice) <= len(files) else 0
    print(f"→ Verwende: {files[idx]}")
    return files[idx]

def main():
    try:
        results_file = load_results_file()
    except FileNotFoundError as e:
        print(e)
        return

    data = load_results(results_file)
    tests = data.get("tests", {})
    for test_id, test in tests.items():
        if "variants" not in test:
            print(f"Test '{test.get('name', test_id)}' hat keine Varianten – überspringe.")
            continue
        variants = test["variants"]
        param_names = list(variants[0]["setup"].keys())[:2]
        all_targets = set()
        for v in variants:
            for exec in v["executions"]:
                all_targets.update(exec["targets"].keys())
        targets = sorted(list(all_targets))
        if not targets:
            print(f"Test '{test['name']}' hat keine Targets – keine Matrixdarstellung möglich.")
            continue
        tensor, param_values, targets_list = build_3d_tensor(variants, param_names, targets)
        plot_3d_matrix(test["name"], tensor, param_values, param_names, targets_list)

if __name__ == "__main__":
    main()
