import subprocess
import os
import sys
import shutil
import time
import ast

MAIN_PATH = os.path.abspath(os.path.join(__file__, "..", "..", "..", ".."))
CURRENT_WORKING_FILE_DIRECTORY = os.path.abspath(os.path.join(__file__))
CURRENT_WORKING_DIRECTORY = os.path.abspath(os.path.join(__file__, ".."))

if MAIN_PATH not in sys.path:
    sys.path.append(MAIN_PATH)

dir_path = os.path.dirname(os.path.realpath(__file__))
for root, dirs, files in os.walk(dir_path):
    sys.path.append(root)

sys.path.insert(0, os.path.abspath('../../../../..'))  # Adjust this path based on the actual project structure to ensure all modules are included.


def delete_folder(path):
    """Delete Folder."""
    try:
        shutil.rmtree(path)
        print(f"Successfully deleted the folder: {path}")
    except FileNotFoundError:
        print(f"The folder does not exist: {path}")
    except PermissionError:
        print(f"Permission denied: Unable to delete the folder: {path}")
    except Exception as e:
        print(f"An error occurred: {e}")


def create_directory(path):
    """Create Directory"""
    try:
        os.makedirs(path, exist_ok=True)
        print(f"Directory created successfully at: {path}")
    except PermissionError:
        print(f"Permission denied: Unable to create directory at: {path}")
    except OSError as e:
        print(f"Failed to create directory: {e}")


def install_packages():
    """Install Packages"""
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'sphinx', 'sphinx-rtd-theme'])


def setup_sphinx(source_dir, project_name):
    """Setup sphinx"""
    conf_file = os.path.join(source_dir, 'source', 'conf.py')
    subprocess.call([
        'sphinx-quickstart',
        source_dir,
        '--sep',
        '--project', project_name,
        '--author', 'AALEKHY',
        '-v', '1.0.0',
        '--release', '1.0.0',
        '--language', 'en',
        '--makefile',
        '--batchfile',
        '--no-use-make-mode',
        '--extensions', 'sphinx.ext.autodoc'
    ])
    print(f"Sphinx project (re)generated at {source_dir}.")


def configure_sphinx(source_dir, project_root):
    """Configure the sphinx"""
    conf_file = os.path.join(source_dir, 'source', 'conf.py')
    with open(conf_file, 'a') as file:
        file.write("\nimport os\nimport sys\nsys.path.insert(0, os.path.abspath('../../../../..'))\n")
        file.write("\nhtml_theme = 'sphinx_rtd_theme'\n")


def build_docs(source_dir, build_dir):
    """Build the Sphinx documentation."""
    import sphinx
    from sphinx.application import Sphinx
    source_path = os.path.join(source_dir, 'source')
    build_path = os.path.join(build_dir, 'html')
    doctree_path = os.path.join(build_dir, 'doctrees')
    sphinx_app = Sphinx(source_path, source_path, build_path, doctree_path, 'html')
    sphinx_app.build(force_all=True)
    print(f"Documentation built successfully at {build_path}.")



def parse_python_script(file_path):
    """Parse a Python script to extract functions and their docstrings."""
    with open(file_path, 'r') as file:
        node = ast.parse(file.read(), filename=file_path)
        functions = []
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.FunctionDef):
                func_info = {
                    'name': child.name,
                    'docstring': ast.get_docstring(child),
                    'args': [a.arg for a in child.args.args],
                    'return': ast.unparse(child.returns) if child.returns else None
                }
                functions.append(func_info)
        return functions


def generate_rst_files(source_dir, module_dir):
    """Generate rst files for each Python module."""
    index_rst_path = os.path.join(source_dir, 'source', 'index.rst')
    with open(index_rst_path, 'w') as index_file:
        index_file.write("Welcome to the documentation!\n")
        index_file.write("=============================\n\n")
        index_file.write(".. toctree::\n")
        index_file.write("   :maxdepth: 2\n\n")
        for root, dirs, files in os.walk(module_dir):
            for file in files:
                if file.endswith('.py') and not file.startswith('__'):
                    module_name = os.path.splitext(file)[0]
                    functions = parse_python_script(os.path.join(root, file))
                    doc_rst_path = os.path.join(source_dir, 'source', f'{module_name}.rst')
                    with open(doc_rst_path, 'w') as doc_file:
                        doc_file.write(f"{module_name}\n")
                        doc_file.write("=" * len(module_name) + "\n\n")
                        doc_file.write(f".. automodule:: {module_name}\n")
                        doc_file.write("   :members:\n")
                        doc_file.write("   :undoc-members:\n")
                        doc_file.write("   :show-inheritance:\n")

                        for func in functions:
                            doc_file.write(f"\n.. autofunction:: {module_name}.{func['name']}\n")
                            if func['docstring']:
                                doc_file.write(f"    {func['docstring']}\n")
                        index_file.write(f"   {module_name}\n")


if __name__ == "__main__":
    project_root = r'C:\Users\aalekhy\PycharmProjects\pythonProject\e2xstreamline-main1 1\e2xstreamline-main11\mye2x'  # Update this to your actual project path
    docs_source = os.path.join(project_root, 'docs')
    docs_build = os.path.join(docs_source, '_build')
    module_dir = os.path.join(project_root, 'e2xostream')  # Update to where your Python modules are

    delete_folder(docs_source)
    time.sleep(5)
    create_directory(docs_source)
    time.sleep(5)
    project_name = 'E2XOStream'  # Update this with your project's name

    install_packages()
    setup_sphinx(docs_source, project_name)
    configure_sphinx(docs_source, project_root)
    generate_rst_files(docs_source, module_dir)
    build_docs(docs_source, docs_build)
