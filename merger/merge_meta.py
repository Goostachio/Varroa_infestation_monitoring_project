import re
from pathlib import Path

NEEDED_KEYS = [
    "EI_CLASSIFIER_PROJECT_ID",
    "EI_CLASSIFIER_PROJECT_OWNER",
    "EI_CLASSIFIER_PROJECT_NAME",
    "EI_CLASSIFIER_PROJECT_DEPLOY_VERSION",
    "EI_CLASSIFIER_NN_INPUT_FRAME_SIZE",
    "EI_CLASSIFIER_RAW_SAMPLE_COUNT",
    "EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME",
    "EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE",
    "EI_CLASSIFIER_INPUT_WIDTH",
    "EI_CLASSIFIER_INPUT_HEIGHT",
    "EI_CLASSIFIER_NN_OUTPUT_COUNT",
    "EI_CLASSIFIER_LABEL_COUNT",
    "EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE",
    "EI_STUDIO_VERSION_MAJOR",
    "EI_STUDIO_VERSION_MINOR",
    "EI_STUDIO_VERSION_PATCH",
]

def _parse_defines(text: str) -> dict:
    d = {}
    for line in text.splitlines():
        m = re.match(r"^\s*#define\s+([A-Z0-9_]+)\s+(.*)\s*$", line)
        if m:
            d[m.group(1)] = m.group(2)
    return d

def _replace_define(lines: list[str], name: str, new_value: str) -> bool:
    pat = re.compile(rf"^(\s*#define\s+{re.escape(name)}\s+).*$")
    for i, line in enumerate(lines):
        m = pat.match(line)
        if m:
            lines[i] = m.group(1) + new_value + ("\n" if not line.endswith("\n") else "\n")
            return True
    return False

def merge_model_metadata(bee_path: str, varroa_path: str, out_path: str) -> None:
    bee_text = Path(bee_path).read_text(encoding="utf-8", errors="replace")
    var_text = Path(varroa_path).read_text(encoding="utf-8", errors="replace")

    bee_defs = _parse_defines(bee_text)
    var_defs = _parse_defines(var_text)

    # Start from Bee file as the output
    out_lines = bee_text.splitlines(True)

    # 1) Make arena size safe for both models (use MAX)
    bee_arena = int(bee_defs["EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE"])
    var_arena = int(var_defs["EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE"])
    max_arena = str(max(bee_arena, var_arena))

    ok = _replace_define(out_lines, "EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE", max_arena)
    if not ok:
        raise RuntimeError("Could not find EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE to replace")

    # 2) Append a namespaced Varroa block (no collisions)
    # Put it near the end, before the final #endif
    insert_idx = None
    for i, line in enumerate(out_lines):
        if line.strip().startswith("#endif"):
            insert_idx = i
            break
    if insert_idx is None:
        raise RuntimeError("Could not find final #endif in output file")

    block = []
    block.append("\n// ---- Secondary model metadata (Varroa) - namespaced to avoid collisions ----\n")
    for k in NEEDED_KEYS:
        if k in var_defs:
            # Convert EI_CLASSIFIER_* -> EI_VARROA_*
            kk = k.replace("EI_CLASSIFIER_", "EI_VARROA_").replace("EI_STUDIO_", "EI_VARROA_STUDIO_")
            block.append(f"#define {kk} {var_defs[k]}\n")
    block.append("// -------------------------------------------------------------------------\n")

    out_lines[insert_idx:insert_idx] = block

    Path(out_path).write_text("".join(out_lines), encoding="utf-8")

# Example:
merge_model_metadata(
    "src-b/src/model-parameters/model_metadata.h",
    "src_v/src/model-parameters/model_metadata.h",
    "out/src/model-parameters/model_metadata.h")
