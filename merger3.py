import os
import re

# Function to strip comments from the C code
def strip_comments(text):
    text = re.sub(r'//.*', '', text)
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    return text

# Function to balance preprocessor directives
def balance_preprocessor_directives(code):
    lines = code.split('\n')
    stack = []
    balanced_code = []
    for line in lines:
        balanced_code.append(line)
        if re.match(r'#\s*if', line) or re.match(r'#\s*ifdef', line) or re.match(r'#\s*ifndef', line):
            stack.append(line)
        elif re.match(r'#\s*endif', line):
            if stack:
                stack.pop()
    
    while stack:
        stack.pop()
        balanced_code.append('#endif')
    
    return '\n'.join(balanced_code)

# Function to extract declarations and definitions
def extract_declarations_and_definitions(code):
    declarations = []
    definitions = []
    lines = code.split('\n')
    decl_re = re.compile(r'^\s*((typedef|extern|#define|struct|union|enum|#ifdef|#ifndef|#endif|#else|volatile|register|inline|static)?\s*[\w\s,*]+\s*\w+\s*\(.*\);\s*|#include\s*[\"<].+?[\">])$')
    func_def_re = re.compile(r'^\s*[\w\s,*]+\s*\w+\s*\(.*\)\s*\{')
    in_func = False
    func_block = []
    for line in lines:
        if decl_re.match(line) and not in_func:
            declarations.append(line)
        elif func_def_re.match(line) and not in_func:
            in_func = True
            func_block.append(line)
        elif in_func:
            func_block.append(line)
            if line.strip() == '}':
                in_func = False
                definitions.extend(func_block)
                func_block = []
        else:
            definitions.append(line)
    return '\n'.join(declarations), '\n'.join(definitions)

# Function to merge files
def merge(file, base_dir, visited=None, missing_includes=None, files_not_found=None, c_cpp_queues=None, base_include_set=None):
    if visited is None:
        visited = set()
    if missing_includes is None:
        missing_includes = set()
    if files_not_found is None:
        files_not_found = []
    if c_cpp_queues is None:
        c_cpp_queues = []
    if base_include_set is None:
        base_include_set = set()

    file_path = os.path.abspath(os.path.join(base_dir, file))
    print(f'Visiting: {file_path}')

    if file_path in visited:
        return '', missing_includes, files_not_found, c_cpp_queues, base_include_set

    visited.add(file_path)
    code = '\n'

    if not os.path.exists(file_path):
        print(f'File not found: {file_path}')
        files_not_found.append(file_path)
        return '', missing_includes, files_not_found, c_cpp_queues, base_include_set

    with open(file_path) as f:
        for line in f:
            match = re.match(r'#include\s*["<](.*?)[">]', line)
            if match:
                inc_file = match.group(1)
                inc_dir = os.path.dirname(file_path)
                inc_path = os.path.join(inc_dir, inc_file)
                
                if not os.path.exists(inc_path):
                    inc_path = os.path.join(base_dir, inc_file)

                if os.path.exists(inc_path):
                    merged_code, merged_includes, merged_files_not_found, merged_c_cpp_queues, merged_base_include_set = merge(
                        inc_file, base_dir, visited, missing_includes, files_not_found, c_cpp_queues, base_include_set
                    )
                    code += strip_comments(merged_code)
                    missing_includes |= merged_includes
                    files_not_found = merged_files_not_found
                    c_cpp_queues = merged_c_cpp_queues
                    base_include_set = merged_base_include_set

                    base, ext = os.path.splitext(inc_file)
                    for comp_ext in ['.c', '.cpp']:
                        comp_file = base + comp_ext
                        comp_path = os.path.join(inc_dir if os.path.exists(os.path.dirname(inc_path)) else base_dir, comp_file)
                        if os.path.exists(comp_path):
                            comp_abs_path = os.path.abspath(comp_path)
                            if comp_abs_path not in visited:
                                c_cpp_queues.append(comp_path)
                else:
                    print(f'Include not found: {inc_path}')
                    missing_includes.add(line.strip())
                    base_include_set.add(line.strip())
                    files_not_found.append(inc_path)
            else:
                code += strip_comments(line)

    return code, missing_includes, files_not_found, c_cpp_queues, base_include_set

# Function to save the merged code
def save_merged_code(entry_file, base_dir, output_file):
    visited = set()
    code, missing_includes, files_not_found, c_cpp_queues, base_include_set = merge(entry_file, base_dir, visited)

    header_declarations, _ = extract_declarations_and_definitions(code)

    c_cpp_declarations = ''
    c_cpp_definitions = ''
    for c_cpp_file in c_cpp_queues:
        print(f'Appending C/CPP file: {c_cpp_file}')
        comp_code, comp_includes, comp_files_not_found, _, _ = merge(os.path.relpath(c_cpp_file, base_dir), base_dir, visited)
        decls, defs = extract_declarations_and_definitions(strip_comments(comp_code))
        c_cpp_declarations += decls + '\n'
        c_cpp_definitions += defs + '\n'
        missing_includes |= comp_includes
        files_not_found.extend(comp_files_not_found)

    code = header_declarations + '\n' + c_cpp_declarations + '\n' + c_cpp_definitions
    balanced_code = balance_preprocessor_directives(code)

    non_empty_code = "\n".join(line for line in balanced_code.split('\n') if line.strip())

    with open(output_file, 'w') as f:
        if base_include_set:
            f.write('\n'.join(sorted(base_include_set)) + '\n')
        f.write(non_empty_code)

    if files_not_found:
        print("\nFiles not found during traversal:")
        for fnf in files_not_found:
            print(fnf)

# Main configuration
entry_file = 'main.cpp'
base_dir = 'input'
output_file = 'output.cpp'
save_merged_code(entry_file, base_dir, output_file)