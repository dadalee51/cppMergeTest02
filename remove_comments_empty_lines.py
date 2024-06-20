import re
import os
import sys

def strip_comments_and_empty_lines(text):
    # Remove single-line comments
    text = re.sub(r'//.*', '', text)
    # Remove multi-line comments
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    # Remove empty lines
    text = os.linesep.join([s for s in text.splitlines() if s.strip()])
    return text

def main():
    if len(sys.argv) != 2:
        print("Usage: python remove_comments_empty_lines.py <file_path>")
        sys.exit(1)

    file_path = sys.argv[1]

    if not os.path.isfile(file_path):
        print(f"File not found: {file_path}")
        sys.exit(1)

    with open(file_path, 'r') as file:
        content = file.read()

    stripped_content = strip_comments_and_empty_lines(content)

    with open(file_path, 'w') as file:
        file.write(stripped_content)

if __name__ == "__main__":
    main()
