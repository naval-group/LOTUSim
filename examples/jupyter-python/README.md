# Jupyter examples

## Using lotusim with jupyter notebook

Every terminal below runs inside `nix develop`, which puts the workspace, the
LOTUSIM_* paths and `xdyn-for-cs` in scope.

1. Run lotusim

In a first terminal, run:
   ```shell
   mise run sim
   ```

2. Run jupyter notebook

The devShell carries no notebook server, so bring one in on the flake's own nixpkgs.
In a second terminal, type:
   ```shell
   nix shell --inputs-from . nixpkgs#python3Packages.jupyter -c jupyter-notebook ${LOTUSIM_PATH}
   ```

3. Run the jupyter example

A page will appear on your browser, navigate to examples/jupyter-python. Click on the jupyter example you want to run and click on the button page at the top of the page.
For the example "controlling_ships.ipynb", before running it, you need to open a new terminal and enter the command below:
   ```shell
   xdyn-for-cs $LOTUSIM_MODELS_PATH/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
   ```