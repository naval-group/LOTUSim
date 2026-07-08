# Python examples

## Using LOTUSim with python

0. Make the example script executable

In a first terminal, run:
   ```shell
   cd lotusim_ws/src/LOTUSim/examples/python-scripts
   chmod +x spawn_ships.py
   chmod +x controlling_ships.py
   ```
This only needs to be done **once**, not every time you want to run the script.

---

### For spawning_ships example

1. Run LOTUSim

In a first terminal, run:
   ```shell
   lotusim run
   ```

2. Source environment

In a second terminal, type the following:
   ```shell
   source ${LOTUSIM_WS}/install/setup.bash
   ```

3. Run the example script

In the second terminal, type the following command:
   ```shell
   python3 spawn_ships.py
   ```

4. Optional: try other examples

Inside 'spawn_ships.py', you can uncomment other spawn commands:
   ```shell
   # future2 = node.spawn_ship_with_dynamics(1)
   # future3 = node.spawn_aerial_drone(1)
   # future4 = node.spawn_circling_ship(2)
   ```

Then re-run the script to experiment with different models.

---

### For controlling_ships example

1. Run XDYN

In a first terminal, run:
   ```shell
   xdyn-for-cs $HOME/lotusim_ws/src/LOTUSim/assets/models/lrauv/lrauv.yml --verbose --address 127.0.0.1 --dt 0.2 --port 12346
   ```

2. Run LOTUSim

In a second terminal, run:
   ```shell
   lotusim run
   ```

3. Run LOTUSim UI

In a third terminal, run:
   ```shell
   lotusim ui
   ```

4. Spawn the ship & send commands

In a fourth terminal, run:
   ```shell
   python3 controlling_ships.py 
   ```

---

## Information about controlling ships with xdyn

When building your own examples, keep the following in mind:

**Vessel naming convention**
When entities are spawned, their names are automatically assigned in the format `model_name_0`, `model_name_1`, ...
The index increments for each new instance.
👉 Make sure you use the correct vessel name when sending commands, otherwise they will not be applied.

**Underwater vs surface behavior (important for XDyn connections)**
XDyn determines whether a vessel is underwater or surface based on its z position:
    z <= -10 → underwater
    z >= 10 → aerial
    otherwise → surface

Each domain (underwater, surface, aerial) uses a different WebSocket connection (URI).
In these examples, vessels are spawned underwater, so only the underwater XDyn connection is used.

⚠️ If your vessel transitions between domains (e.g., from surface at z = 0 to underwater at z = -100), or if you spawn vessels in different domains, you must:
   -  run XDyn instances for each relevant domain, and
   -  ensure each one is listening on the correct URI/port.

