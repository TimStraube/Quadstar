import json
import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def load_results_file():
    results_dir = "results"
    files = [os.path.join(results_dir, f) for f in os.listdir(results_dir) if f.startswith("test_results") and f.endswith(".json")]
    if not files:
        raise FileNotFoundError("Keine test_results_*.json Datei im 'results'-Ordner gefunden!")
    print("\nVerfügbare Ergebnisdateien:")
    for i, f in enumerate(files, 1):
        print(f"{i}: {f}")
    choice = input("\nDatei wählen (Nummer): ").strip()
    idx = int(choice) - 1 if choice.isdigit() and 1 <= int(choice) <= len(files) else 0
    print(f"→ Verwende: {files[idx]}")
    return files[idx]

def load_test_results(filename):
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    # Wenn "tests" existiert, nimm das, sonst das ganze Objekt
    return data["tests"] if "tests" in data else data

def extract_data_for_analysis(tests):
    records = []
    for test_id, test_content in tests.items():
        if "variants" not in test_content:
            continue  # Nur Ergebnisdaten verarbeiten
        for variant in test_content["variants"]:
            setup = variant["setup"]
            for execution in variant["executions"]:
                for target, tdata in execution["targets"].items():
                    status = 1 if tdata["status"] == "bestanden" else 0
                    for param, value in setup.items():
                        records.append({
                            "target": target,
                            "setup_param": param,
                            "setup_value": value,
                            "status": status
                        })
    return pd.DataFrame(records)

def compute_dependency_strength(df):
    strength = []
    grouped = df.groupby(["target", "setup_param", "setup_value"])["status"].mean().reset_index()
    for (target, param), sub in grouped.groupby(["target", "setup_param"]):
        dep_strength = sub["status"].max() - sub["status"].min()
        strength.append({"target": target, "setup_param": param, "dependency_strength": dep_strength})
    return pd.DataFrame(strength)

def plot_heatmap(df_strength):
    pivot = df_strength.pivot(index="target", columns="setup_param", values="dependency_strength")
    plt.figure(figsize=(8, max(3, len(pivot)*0.6)))
    sns.heatmap(pivot, annot=True, cmap="YlOrRd", linewidths=0.5, cbar_kws={'label': 'Abhängigkeit'})
    plt.title("Abhängigkeit der Testziele von Setup-Parametern", fontsize=14, fontweight="bold")
    plt.xlabel("Setup-Parameter")
    plt.ylabel("Testziel")
    plt.tight_layout()
    plt.show()

def main():
    filename = load_results_file()
    tests = load_test_results(filename)
    df = extract_data_for_analysis(tests)
    # Prüfe, ob Ergebnisdaten vorhanden sind
    required_cols = {"target", "setup_param", "setup_value", "status"}
    if df.empty or not required_cols.issubset(df.columns):
        print("\nDie gewählte Datei enthält keine Ergebnisdaten oder ist kein Test-Resultat!")
        print("Bitte wähle eine Datei aus, die echte Testresultate enthält (z.B. aus dem results-Ordner).")
        return
    df_strength = compute_dependency_strength(df)
    print("\nBerechnete Abhängigkeiten:")
    print(df_strength.sort_values(by=["target", "setup_param"]))
    plot_heatmap(df_strength)

if __name__ == "__main__":
  main()