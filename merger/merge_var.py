#!/usr/bin/env python3
import re
from pathlib import Path

BEE_ID = "872791"
VARROA_ID = "874563"

def strip_guard_and_trailer(text: str) -> str:
    # remove leading guard block
    text = re.sub(r'^\s*#ifndef\s+_EI_CLASSIFIER_MODEL_VARIABLES_H_\s*\n'
                  r'\s*#define\s+_EI_CLASSIFIER_MODEL_VARIABLES_H_\s*\n', '',
                  text, flags=re.MULTILINE)
    # remove final endif line (and comment)
    text = re.sub(r'\n\s*#endif\s*//\s*_EI_CLASSIFIER_MODEL_VARIABLES_H_\s*\n?\s*$',
                  '\n', text, flags=re.MULTILINE)
    return text

def remove_generic_aliases(text: str) -> str:
    # Remove the 4 generic aliases at bottom (they collide across models)
    patterns = [
        r'^\s*ei_impulse_handle_t&\s+ei_default_impulse\s*=\s*.*?;\s*\n',
        r'^\s*constexpr\s+auto&\s+ei_classifier_inferencing_categories\s*=\s*.*?;\s*\n',
        r'^\s*const\s+auto\s+ei_dsp_blocks_size\s*=\s*.*?;\s*\n',
        r'^\s*ei_model_dsp_t\s*\*\s*ei_dsp_blocks\s*=\s*.*?;\s*\n',
    ]
    for pat in patterns:
        text = re.sub(pat, '', text, flags=re.MULTILINE)
    return text

def main(bee_path: Path, varroa_path: Path, out_path: Path):
    bee = bee_path.read_text(encoding="utf-8")
    varroa = varroa_path.read_text(encoding="utf-8")

    # strip guards from both, then rebuild one new guard
    bee_body = strip_guard_and_trailer(bee)
    varroa_body = strip_guard_and_trailer(varroa)

    # keep bee's generic aliases (so existing bee code still works),
    # but remove varroa's generic aliases (to avoid collisions)
    varroa_body = remove_generic_aliases(varroa_body)

    # Merge: insert varroa right before the end of bee file (now already stripped)
    merged = []
    merged.append("/* AUTO-MERGED: bee + varroa model_variables.h */\n")
    merged.append("#ifndef _EI_CLASSIFIER_MODEL_VARIABLES_MERGED_H_\n")
    merged.append("#define _EI_CLASSIFIER_MODEL_VARIABLES_MERGED_H_\n\n")

    merged.append(bee_body.rstrip() + "\n\n")
    merged.append("\n/* ===== VARROA MODEL APPEND ===== */\n\n")
    merged.append(varroa_body.rstrip() + "\n\n")

    # Add explicit named handles so you can call either model easily
    merged.append("/* ===== EXPLICIT HANDLE ALIASES (NON-COLLIDING) ===== */\n")
    merged.append(f"static inline ei_impulse_handle_t& ei_bee_impulse = impulse_handle_{BEE_ID}_1;\n")
    merged.append(f"static inline ei_impulse_handle_t& ei_varroa_impulse = impulse_handle_{VARROA_ID}_1;\n\n")

    merged.append("#endif // _EI_CLASSIFIER_MODEL_VARIABLES_MERGED_H_\n")

    out_path.write_text("".join(merged), encoding="utf-8")
    print(f"Wrote: {out_path}")

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--bee", required=True, help="path to bee model-parameters/model_variables.h")
    ap.add_argument("--varroa", required=True, help="path to varroa model-parameters/model_variables.h")
    ap.add_argument("--out", required=True, help="output merged header path")
    args = ap.parse_args()
    main(Path(args.bee), Path(args.varroa), Path(args.out))


# python3 merge_var.py \
#   --bee src-b/src/model-parameters/model_variables.h \
#   --varroa src_v/src/model-parameters/model_variables.h \
#   --out out/src/model-parameters/model_variables.h
