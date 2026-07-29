# Jupyter examples

## Using lotusim with jupyter notebook

1. Run lotusim

In a first terminal, run:
   ```shell
   lotusim run
   ```

2. source environment

In a second terminal, type the following:
   ```shell
   source ${LOTUSIM_WS}/install/setup.bash
   ```

3. Run jupyter notebook

Then type this command in the second terminal:
   ```shell
   jupyter-notebook ${LOTUSIM_PATH}
   ```

4. Run the jupyter example

A page will appear on your browser, navigate to examples/jupyter-python. Click on the jupyter example you want to run and click on the button page at the top of the page.
For the example "controlling_ships.ipynb", before running it, you need to open a new terminal and enter the command below:
   ```shell
   xdyn-for-cs $HOME/lotusim_ws/src/LOTUSim/assets/models/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
   ```