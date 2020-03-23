import matplotlib.pyplot as plt
import numpy as np
import math


class Node:
    def __init__(self):
        self.infected_since = None
        self.last_send = 0

    def is_infected(self, step_idx):
        return self.infected_since is not None and step_idx >= self.infected_since

class Simulation:
    def __init__(self, num_nodes, frequency, fanout, mode):
        self.num_nodes = num_nodes
        self.nodes = [Node() for _ in range(num_nodes)]
        self.fanout = fanout
        self.frequency = frequency
        self.base_tick = 1000  # Simulate milliseconds
        self.mode = mode

        # Initialize an array of indices to pick peers from.
        # `num_nodes-1` because nodes don't send updates to themselves
        self.bag = [i for i in range(self.num_nodes - 1)]

        if mode == "bulk":
            self.interval = (1.0 / self.frequency) * self.base_tick
        elif mode == "continuous":
            self.interval = (1.0 / (self.frequency * self.fanout)) * self.base_tick
            self.fanout = 1
        else:
            assert False

        self.step_idx = 0

        self.nodes[0].infected_since = 0

        self.num_infected = []
        self.x_ticks = []

    def run(self):
        while True:
            if np.alltrue([node.is_infected(self.step_idx) for node in self.nodes]):
                break

            for node_idx, node in enumerate(self.nodes):
                if not node.is_infected(self.step_idx):
                    continue

                if self.step_idx >= node.last_send + self.interval:
                    node.last_send += self.interval

                    peers = np.random.choice(self.bag, size=self.fanout)
                    # Make sure nodes are not sending messages to themselves by increasing indices
                    # greater or equal to `node_idx` by one.
                    peers = [peer_idx if peer_idx < node_idx else peer_idx + 1 for peer_idx in peers]

                    for peer_idx in peers:
                        if self.nodes[peer_idx].infected_since is None:
                            self.nodes[peer_idx].infected_since = self.step_idx + msg_delay_steps
                            self.nodes[peer_idx].last_send = self.step_idx + msg_delay_steps
                            print(f"Node {node_idx} infected_since node {peer_idx}")

                # Asserts that base_tick is fast enough
                assert self.step_idx < node.last_send + self.interval

            self.num_infected.append(np.count_nonzero([node.is_infected(self.step_idx) for node in self.nodes]))
            self.x_ticks.append(float( self.step_idx) / base_tick)
            self.step_idx += 1


num_nodes = 128
fanout = 4
frequency = 1
base_tick = 1000  # Simulate milliseconds
msg_delay_steps = 0

step_idx = 0

num_infected_theory = []
num_infected_naive = []
x_ticks_theory = []

while True:
    beta = float(fanout) / (num_nodes - 1)
    t = float(step_idx) / base_tick
    num_infected_theory.append(float(num_nodes) / (1 + (num_nodes - 1) * math.exp(-beta * num_nodes * t)))

    num_infected_naive.append(fanout ** (1 + (float(step_idx) / base_tick) * frequency))

    x_ticks_theory.append(float(step_idx) / base_tick)

    step_idx += 1

    if num_infected_naive[-1] + 0.1 > num_nodes and num_infected_theory[-1] + 0.1 > num_nodes:
        break

bulk_simulation = Simulation(num_nodes=num_nodes, frequency=frequency, fanout=fanout, mode="bulk")
bulk_simulation.run()

continuous_simulation = Simulation(num_nodes=num_nodes, frequency=frequency, fanout=fanout, mode="continuous")
continuous_simulation.run()

plt.plot(bulk_simulation.x_ticks, bulk_simulation.num_infected, label="Bulk fanout")
plt.plot(continuous_simulation.x_ticks, continuous_simulation.num_infected, label="Continuous sending")
plt.plot(x_ticks_theory, num_infected_theory, label="Theory")
plt.plot(x_ticks_theory, num_infected_naive, label="Naive")
plt.legend()
plt.show()
