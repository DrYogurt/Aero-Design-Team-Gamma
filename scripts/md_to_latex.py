#!/usr/bin/env python3
import os
import yaml
import re
import subprocess
import sys
from pathlib import Path

def load_variables():
    """Load global variables from YAML file"""
    with open('global_variables/variables.yaml', 'r') as f:
        return yaml.safe_load(f)

def replace_variables(content, variables):
    """Replace variable placeholders in content with actual values"""
    def replace_match(match):
        var_path = match.group(1).split('.')
        value = variables
        for key in var_path:
            value = value[key]
        return str(value)
    
    pattern = r'{{\s*([\w.]+)\s*}}'
    return re.sub(pattern, replace_match, content)

def convert_markdown_to_latex(md_path, template_path, variables):
    """Convert markdown to LaTeX using pandoc and custom template"""
    with open(md_path, 'r') as f:
        content = f.read()
    
    # Replace variables in content
    content_with_vars = replace_variables(content, variables)
    
    # Write to temporary file
    temp_md = md_path.with_suffix('.temp.md')
    with open(temp_md, 'w') as f:
        f.write(content_with_vars)
    
    # Convert to LaTeX using pandoc
    output_tex = md_path.with_suffix('.tex')
    subprocess.run([
        'pandoc',
        '-f', 'markdown',
        '-t', 'latex',
        '--template', template_path,
        '-o', str(output_tex),
        str(temp_md)
    ])
    
    # Cleanup temporary file
    temp_md.unlink()
    
    return output_tex

def compile_latex_to_pdf(tex_path):
    """Compile LaTeX to PDF using pdflatex"""
    working_dir = tex_path.parent
    subprocess.run([
        'pdflatex',
        '-interaction=nonstopmode',
        str(tex_path.name)  # Use just filename since we're changing directory
    ], cwd=working_dir)  # Set working directory to the input file's location
    
    # Clean up auxiliary files
    for ext in ['.aux', '.log', '.nav', '.out', '.snm', '.toc']:
        aux_file = working_dir / f"{tex_path.stem}{ext}"  # Proper path concatenation
        if aux_file.exists():
            aux_file.unlink()

def main():
    # Load global variables
    variables = load_variables()
    
    # Process weekly folder
    week_folder = Path(sys.argv[1])
    
    # Convert and compile report
    report_md = week_folder / 'report.md'
    report_template = Path('templates/report_template.tex')
    if report_md.exists():
        tex_file = convert_markdown_to_latex(report_md, report_template, variables)
        compile_latex_to_pdf(tex_file)
    
    # Convert and compile presentation
    presentation_md = week_folder / 'presentation.md'
    presentation_template = Path('templates/presentation_template.tex')
    if presentation_md.exists():
        tex_file = convert_markdown_to_latex(presentation_md, presentation_template, variables)
        compile_latex_to_pdf(tex_file)

if __name__ == '__main__':
    main()
