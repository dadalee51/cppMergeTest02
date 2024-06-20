import os
import re

def strip_comments(text):
    text = re.sub(r'//.*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text

def merge(file, base_dir, visited=None, missing_includes=None, files_not_found=None, base_include_set=None):
    if visited is None:
        visited = set()
    if missing_includes is None:
        missing_includes = set()
    if files_not_found is None:
        files_not_found = []
    if base_include_set is None:
        base_include_set = set()

    file_path = os.path.abspath(os.path.join(base_dir, file))
    print(f'Visiting: {file_path}')

    if file_path in visited:
        return '', missing_includes, files_not_found, base_include_set

    visited.add(file_path)
    code = ''

    if not os.path.exists(file_path):
        print(f'File not found: {file_path}')
        files_not_found.append(file_path)
        return '', missing_includes, files_not_found, base_include_set

    with open(file_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            match = re.match(r'#include\s*["<](.*?)[">]', line)
            if match:
                inc_file = match.group(1)
                inc_dir = os.path.dirname(file_path)
                inc_path = os.path.join(inc_dir, inc_file)
                
                if not os.path.exists(inc_path):
                    inc_path = os.path.join(base_dir, inc_file)

                if os.path.exists(inc_path):
                    merged_code, merged_includes, merged_files_not_found, merged_base_include_set = merge(
                        inc_file, base_dir, visited, missing_includes, files_not_found, base_include_set
                    )
                    code += '\n' + strip_comments(merged_code) + '\n'
                    missing_includes |= merged_includes
                    files_not_found = merged_files_not_found
                    base_include_set = merged_base_include_set

                    base, ext = os.path.splitext(inc_file)
                    for comp_ext in ['.c', '.cpp']:
                        comp_file = base + comp_ext
                        comp_path = os.path.join(inc_dir if os.path.exists(os.path.dirname(inc_path)) else base_dir, comp_file)
                        if os.path.exists(comp_path):
                            comp_abs_path = os.path.abspath(comp_path)
                            if comp_abs_path not in visited:
                                comp_code, comp_includes, comp_files_not_found, comp_base_include_set = merge(
                                    comp_file, base_dir, visited, missing_includes, files_not_found, base_include_set
                                )
                                code += '\n' + strip_comments(comp_code) + '\n'
                                missing_includes |= comp_includes
                                files_not_found = comp_files_not_found
                                base_include_set = comp_base_include_set
                else:
                    print(f'Include not found: {inc_path}')
                    missing_includes.add(line.strip())
                    base_include_set.add(line.strip())
                    files_not_found.append(inc_path)
            else:
                code += '\n' + strip_comments(line)

    return code, missing_includes, files_not_found, base_include_set

def save_merged_code(entry_file, base_dir, output_file):
    visited = set()
    code, missing_includes, files_not_found, base_include_set = merge(entry_file, base_dir, visited)

    non_empty_code = "\n".join(line for line in code.split('\n') if line.strip())

    with open(output_file, 'w') as f:
        if base_include_set:
            f.write('\n'.join(sorted(base_include_set)) + '\n')
        f.write(non_empty_code)

    if files_not_found:
        print("\nFiles not found during traversal:")
        for fnf in files_not_found:
            print(fnf)

entry_file = 'main.cpp'
base_dir = 'input'
output_file = 'output.cpp'
save_merged_code(entry_file, base_dir, output_file)
