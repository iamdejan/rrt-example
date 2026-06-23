# AGENTS.md

## Code Generation

Hi agent! Whatever model you are using, make sure you adhere to the guide on this file to generate the code.

### Rules of engagement

1. You're allowed to create functions and classes to generate the code.
2. With the exception of import statements, explain the flow of the program using comments. Make sure that you write not only what the program is doing, but why. It will help me to judge your work.
3. You have to add documentation for functions and classes. The documentation should follow NumPy style. The documentation should contain what is the function for (basically the description), a brief summary of the steps, and input and output parameters. If your function has the ability to throw error, please state it in the documentation as well.

### Debugging

In this repository, we don't have Python installed in local machine. Instead, Python executable is managed by Pixi, which is why in [pixi.toml](pixi.toml) under `[dependencies]` section you will have `python` listed as dependency.

The consequence is that every Python command you want to run should be run under `pixi run` command. I will list some of the examples here:
- Check Python version: Instead of `python3 --version`, you should run `pixi run python3 --version`.

## Code Validation

Hi agent! Whatever model you are using, make sure you validate the code you generated.

### Steps to Validate

Run these commands in sequence:
1. `pixi run lint-ci`: ensure no linter errors.
2. `pixi run lint-fix`: if there is any linter error, fix it with this command.
3. `pixi run lint-ci`: recheck again, maybe there are linter errors that need manual fix.
4. `pixi run type-check`: ensure no type errors.

After the validation is finished, update the project tree structure and file descriptions in README.md if needed. This is to ensure we always have updated documentation.
