#!/usr/bin/env python3
"""count_lines_project.py

Parcourt récursivement un dossier, ouvre tous les fichiers .py (y compris dans les sous-dossiers)
et affiche le nombre total de lignes dans le projet.

Usage:
    python count_lines_project.py /chemin/vers/projet [--exclude EX1 EX2] [--details] [--no-empty] [--only-code]

Options:
  --exclude DIR     Répéter pour exclure des dossiers par nom (ex: --exclude venv .venv build)
  --details         Affiche le nombre de lignes par fichier
  --no-empty        Ne compte pas les lignes vides (mais compte les lignes commentaires)
  --only-code       Ne compte que les lignes non vides et non-commentaires

Comportement:
  - Par défaut le script compte toutes les lignes de tous les .py trouvés.
  - Les dossiers suivants sont ignorés automatiquement: __pycache__, .git
  - Lecture robuste: essai utf-8 puis latin-1 si nécessaire.
"""

from pathlib import Path
import argparse
import sys
import os
from typing import Tuple, Dict

DEFAULT_IGNORED_DIRS = {"__pycache__", ".git"}


def count_lines_in_file(path: Path) -> Tuple[int, int, int]:
    """Retourne (total_lines, non_empty_lines, code_lines)

    - total_lines : nombre de lignes (inclut vides et commentaires)
    - non_empty_lines : lignes non vides (exclut les lignes qui deviennent "" après strip())
    - code_lines : lignes non vides ne commençant pas par un commentaire (#)
    """
    total = 0
    non_empty = 0
    code = 0
    text = None
    # lire en essayant utf-8 puis latin-1
    for enc in ("utf-8", "latin-1"):
        try:
            with path.open("r", encoding=enc) as f:
                text = f.readlines()
            break
        except Exception:
            text = None
    if text is None:
        # impossible de lire le fichier: l'ignorer
        return 0, 0, 0

    for ln in text:
        total += 1
        s = ln.strip()
        if s != "":
            non_empty += 1
            # considérer comme commentaire si commence par # (après strip)
            if not s.startswith("#"):
                code += 1
    return total, non_empty, code


def should_exclude(path: Path, exclude_names) -> bool:
    parts = {p.name for p in path.parents} | {path.name}
    return bool(parts & set(exclude_names))


def main(argv=None):
    p = argparse.ArgumentParser(description="Compte les lignes .py dans un projet (récursif)")
    p.add_argument("root", nargs="?", default=".", help="Dossier racine (défaut: .)")
    p.add_argument("--exclude", "-e", nargs="*", default=[], help="Noms de dossiers à exclure (ex: venv .venv build)")
    p.add_argument("--details", action="store_true", help="Afficher le détail par fichier")
    p.add_argument("--no-empty", action="store_true", help="Ne pas compter les lignes vides")
    p.add_argument("--only-code", action="store_true", help="Ne compter que les lignes non vides et non-commentaires")
    args = p.parse_args(argv)

    root = Path(args.root).resolve()
    if not root.exists():
        print(f"Erreur: '{root}' n'existe pas.")
        sys.exit(2)

    exclude_names = set(args.exclude) | DEFAULT_IGNORED_DIRS

    py_files = list(root.rglob("*.py"))
    # filtrer les fichiers dont l'un des parents ou le nom est dans exclude_names
    py_files = [f for f in py_files if not should_exclude(f, exclude_names)]

    total_lines = 0
    total_non_empty = 0
    total_code = 0
    per_file: Dict[Path, Tuple[int, int, int]] = {}

    for f in sorted(py_files):
        t, ne, c = count_lines_in_file(f)
        total_lines += t
        total_non_empty += ne
        total_code += c
        per_file[f] = (t, ne, c)

    # Choix de l'affichage final selon les flags
    if args.only_code:
        chosen_label = "code_lines"
        chosen_value = total_code
    elif args.no_empty:
        chosen_label = "non_empty_lines"
        chosen_value = total_non_empty
    else:
        chosen_label = "total_lines"
        chosen_value = total_lines

    print("\nRésumé :")
    print(f"  Dossier racine : {root}")
    print(f"  Fichiers .py trouvés : {len(py_files)}")
    print(f"  Lignes totales : {total_lines}")
    print(f"  Lignes non vides : {total_non_empty}")
    print(f"  Lignes de code (non vides, non-commentaires) : {total_code}")
    print(f"\n  Valeur retournée selon options -> {chosen_label} = {chosen_value}\n")

    if args.details:
        print("Détail par fichier :\n")
        for f, (t, ne, c) in per_file.items():
            print(f"{f} : total={t:5d} | non_empty={ne:5d} | code={c:5d}")


if __name__ == "__main__":
    main()
