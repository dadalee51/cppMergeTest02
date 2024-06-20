import os
import re
import sys

def read_file(fp):
    try:
        with open(fp, 'r') as f:
            return f.read()
    except FileNotFoundError:
        print(f"File not found: {fp}")
        return ""

def rmv_comments(text):
    return re.sub(r'//.*?$|/\*.*?\*/', '', text, flags=re.DOTALL | re.MULTILINE)

def fltr_include(line):
    match = re.match(r'#include\s+[<"](.+)[>"]', line)
    return match.group(1) if match else None

def find_file(name, files):
    for file in files:
        if file.endswith(name):
            return file
    return None

def grd_files(dir):
    files = []
    for root, _, fs in os.walk(dir):
        for f in fs:
            files.append(os.path.relpath(os.path.join(root, f), dir))
    return files

def fmt_include(fp):
    return f'#include "{os.path.basename(fp)}"'

def class_decl_statements(content):
    return set(re.findall(r'\bclass\s+(\w+)', content))

input_dir = 'input'
output = 'output.cpp'
main_cpp = 'main.cpp'

if len(sys.argv) > 1:
    main_cpp = sys.argv[1]

all_files = grd_files(input_dir)
visited = set()
unique_incl_names = set()  # Store only filenames to identify unique includes
missing_incl = set()
class_decls = set()

def merge_file(fp, output, files):
    if fp in visited:
        return
    visited.add(fp)
    print(f"Processing: {fp}")

    content = read_file(os.path.join(input_dir, fp))
    if not content:
        return

    # Updating class declarations
    class_decls.update(class_decl_statements(content))

    lines = content.splitlines()
    for line in lines:
        incl = fltr_include(line)
        if incl:
            if incl in unique_incl_names:
                continue  # Skip already processed includes

            incl_path = find_file(incl, files)
            if incl_path:
                print(f"Merging include: {incl_path}")
                unique_incl_names.add(incl)  # Add only the filename to the set
                merge_file(incl_path, output, files)
                    
                if incl.endswith('.h'):
                    src_file = incl[:-2] + '.cpp'
                    src_path = find_file(src_file, files)
                    if src_path:
                        print(f"Merging source: {src_path}")
                        merge_file(src_path, output, files)
            else:
                print(f"Missing include: {incl}")
                missing_incl.add(incl)
        else:
            output.append(line)

output_content = []
main_file_path = find_file(main_cpp, all_files)
if main_file_path:
    merge_file(main_file_path, output_content, all_files)
else:
    print(f"Main file '{main_cpp}' not found in '{input_dir}'.")

# Combine and filter content
result_content = []
merged_content = rmv_comments('\n'.join(output_content))

# Postprocess to remove unnecessary include directives
postprocessed_output = []
for line in merged_content.splitlines():
    line = line.strip()
    if line:
        incl = fltr_include(line)
        if incl:
            header_content = read_file(os.path.join(input_dir, find_file(incl, all_files) or ""))
            header_class_decls = class_decl_statements(header_content)
            if incl in unique_incl_names or not header_class_decls.isdisjoint(class_decls):
                continue  # Skip the include directive if the classes are already declared or merged
        postprocessed_output.append(line)

with open(output, 'w') as f:
    # Write missing includes
    missing_includes_str = '\n'.join(fmt_include(inc) for inc in missing_incl)
    f.write(missing_includes_str + '\n')
    print("\nMissing Includes:\n" + missing_includes_str + "\n")
    
    # Write unique includes
    unique_includes_str = '\n'.join(fmt_include(inc) for inc in unique_incl_names)
    f.write(unique_includes_str + '\n')
    print("\nUnique Includes:\n" + unique_includes_str + "\n")
    
    # Write final processed content
    final_output = '\n'.join(postprocessed_output)
    f.write(final_output)
    print("\nFinal Output:\n" + final_output + "\n")