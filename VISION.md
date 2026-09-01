## LOTUSim Vision

LOTUSim is the first open-source simulation tool released by a major player in French naval defence. Its purpose is to federate research and development around multi-agent
naval simulation, across both military and civil domains.

This document sets out where LOTUSim stands today and where it is heading. It complements, and does not replace, the contribution rules.
How to contribute: [`CONTRIBUTING.md`](CONTRIBUTING.md)

LOTUSim was born from the work of Naval Group PhD students, in France and around the world, at IRL CROSSING in Australia and at Naval Group Far East in Singapore. Together we decided to offer a shared environment so that PhD students could focus on their research rather than on coding a simulation bench. What started with PhD students quickly proved useful well beyond them : to research engineers,algorithm designers and R&D teams at large.

That is the mission: to make the work of algorithm designers, research engineers,
human-factors specialists and roboticists simpler and more effective, by giving them a common playground for their work.

## Current Focus

The priorities for the current period are:

- A first set of open, non-sensitive platform models - civil drones such as RexROV, BlueROV and BlueBoat - made available to the R&D community.
- Basic public sensor models, such as radar and sonar.
- A graphical interface to set up a scenario easily, without manual scripting.

These open models are a deliberate choice. LOTUSim exists so that the community does not have to confine itself to a handful of in-house models: we open what can be opened, and we invite others to bring the rest.

The graphical interface is the most visible part of this period. Users can now browse and launch existing scenarios directly from the interface, place vessels on a map, and
configure a scenario without writing scripts. Full in-GUI scenario authoring will follow in a later release.

## Next

Two parallel directions guide what comes next.

**A scenario library and a reworked API : LOTUSim as a data factory.**
We want a library of civil and military scenarios that can be launched end-to-end with a single action, backed by a well-designed API. The goal is to turn LOTUSim into a
simple-to-use data factory: a way to generate test data for R&D at scale. Alongside this, we are working on the automatic generation of new scenarios, driven by LLM and MCP, for example, describing a scenario in natural language to an agent that translates it into an executable LOTUSim configuration and launches the full simulation.

**An AI-augmented digital twin.**
We are pursuing a digital twin built on mixed physical/AI methods: surrogate models, physics-informed neural networks and related approaches, to accelerate the performance of our models in compute time and, where relevant, in accuracy, across environments, sensors, power management and beyond.

These two axes are our internal focus. We strongly encourage the community to build alongside us: contribute your own platform and sensor models, and your own scenarios, to enrich the shared bank.

## Architecture & Technology Stack

LOTUSim's technical choices serve three objectives: openness (open-source, built on open standards), modularity (add or swap model, sensors and algorithms) and flexibility (configure the simulation to fit your need). The core, governed by Naval Group, holds the architecture, the interfaces and the foundational bricks, and we remain firm there: the main directions are ours to set and to carry.
Around that core, models, sensors and scenarios are open to community contribution.

- **ROS2** - the de facto standard of robotics. It makes LOTUSim naturally compatible
  with the real software bricks roboticists work with, so that algorithms built in
  simulation transpose toward hardware.
- **Gazebo** - a mature open-source robotics simulator providing physics and sensor
  rendering, with native ROS2 integration.
- **xdyn** - an open-source (EPL-2.0) hydrodynamics engine developed by Sirehna. It solves the equations of motion for surface and underwater vehicles under real sea conditions, bringing a level of physical fidelity a generic robotics simulator does not cover - a differentiating brick for the naval domain.
- **C++** - the the core language, chosen for performance and real-time execution, long favoured by industry for demanding simulation. 
- **Python** - the accessible layer on top : the language of algorithm designers and the R&D/AI community, with ready examples for users.

MCP support follows the same logic of openness: exposing LOTUSim so that AI agents can orchestrate it - for instance, going from a natural-language scenario description to a
running simulation.

## Contributing & Scope

The full contribution process lives in [`CONTRIBUTING.md`](CONTRIBUTING.md). A few points of scope are worth stating here, as they shape what fits the project.

- **Proprietary names.** Contributions must not expose the names of proprietary platforms or sensors without agreement. Respect each organisation's wishes about being named or not - some choose to be credited, others do not.
- **AI-generated code.** Treat it with care. Understand, review and verify what you submit. Do not pour in thousands of lines doing forty things at once - one focused change at a time, as set out in [`CONTRIBUTING.md`](CONTRIBUTING.md).

This is a guardrail for a project that means to stay open and modular, not an exhaustive rulebook. Strong rationale and strong community demand can move it.

## References

- Contribution rules: [`CONTRIBUTING.md`](CONTRIBUTING.md)
- License: EPL-2.0 - every contribution is made under this license.
- Repository: [`naval-group/LOTUSim`](https://github.com/naval-group/LOTUSim/)
- Roadmap and product questions: the Product Owner - lotusim_support@naval-group.com
