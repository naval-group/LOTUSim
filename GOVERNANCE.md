# LOTUSim Contribution Integration Process

## Preamble

The purpose of this document is to describe the process for integrating new developments into the public GitHub repository and the private GitLab repository of LOTUSim.

By definition, developments carried out on the **simulation core** must not be of a sensitive nature and must therefore be shareable with all roles defined in this document, as the simulation core is **open source under the EPL 2.0 license**.

### Out of scope (to be addressed later)

The following elements are not covered by this document and will be the subject of dedicated studies:

- code testing and validation processes,

- management process for sensitive models and algorithms,

- support process via `lotusim.support@naval-group.com`.

## Glossary

- **NGSA**: Naval Group SA parent company.

- **Project Team**: LOTUSim team composed of NGSA employees and its subsidiaries (including Centers of Expertise and Sirehna), carrying out developments on the **Private Repository**.

- **Private Repository**: private GitLab repository hosted by the external Digital Factory, containing all developments carried out and integrated by the Project Team.

- **Public Repository**: public GitHub repository accessible to the entire open source community and maintained by the Project Team.

## Overall Organization

LOTUSim developments are coordinated by the Project Team, which interacts with the open source community through:

- the public GitHub repository,

- the support email address `lotusim.support@naval-group.com`,

- monthly newsletters.

The **Project Team** works according to an agile methodology inspired by the **Scrum Guide (November 2020)**:

- daily meetings are replaced by weekly meetings,

- sprint duration is set to **4 weeks**.

The branch management strategy for both repositories follows the best practices recommended by the **GitFlow workflow**:
`https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow`

## Roles

### Product Owner (internal)

- Key interface between users and developers.

- Defines functional requirements and priority features

- Ensures overall project consistency.

- Primary point of contact for external users via `lotusim.support@naval-group.com`.

### Contributor (external)

- Open source contributor or partner using and modifying the software.

- Originator of requirements through the submission of `Issue` on the **Public repository**.

- Develops contributions on their own `Fork` of the project.

### Maintainer (internal)

- Developer with full access rights to the **Private repository**.

- Distributed across NGSA and its subsidiaries.

- Carries out developments defined during sprint plannings.

- Manages the integration of open source contributions.

#### Scrum Master (internal)

- Organizes and moderates Scrum meetings in line with the Product Owner's roadmap.

- Assists the maintainers in meeting their delivery goals.

- Identifies and helps remove blockers that impact the maintainers.

#### Project Maintainer (internal)

Among the maintainers, the **Project Maintainer**:

- has final responsibility for validating `Merge Request`,

- ensures overall technical and architectural consistency of the project.

As LOTUSim evolves, **maintainers by functional scope** may be appointed under the coordination of the **Project Maintainer**.

## Feature Development Process by the Project Team

### Hotfix

Urgent correction of a critical malfunction affecting the stability, execution, or integrity of the simulation core results.

#### Process

- Priority assignment to a maintainer, including during an ongoing sprint.

- Integration via a dedicated `Merge Request`.

- Final validation by the Project Maintainer.

### Bug

Correction of an identified malfunction in an existing feature whose impact is not blocking and can be handled within the normal sprint cycle.

#### Process

- Logged as an Issue.

- Prioritized by the Project Team during sprint planning.

- Fixed within the normal sprint cycle.

- Integration via a dedicated `Merge Request`.

- Final validation by the **Project Maintainer**.

### Contribution

Non-corrective development intended to enhance LOTUSim, including the addition of new features, new models, functional or technical improvements, performance optimizations, or data contributions.

#### Process

- During sprint planning, the **Product Owner** presents the **Sprint Objective**.

- This objective is broken down by the **Scrum Master** into **Backlog Items** and assigned to the **Project Team**.

- Items are decomposed into technical contributions assigned to **Maintainers**.

- Integration via a dedicated `Merge Request`.

- Final validation by the **Project Maintainer**.

### Sensitive Contributions

- Process to be defined.

## Feature Integration Process by the Open Source Community

The detailed process is described in the `CONTRIBUTING.md` file of the **Public Repository**.

Main steps:

1. The contributor formalizes their proposal through an `Issue` on the **Public Repository**.

2. The contributor forks the project and implements the proposed changes.

3. The contributor submits a `Merge Request`.

The **Project Team** commits to providing a reasoned response within one month.
The evaluation may take place during weekly meetings, sprint plannings, or sprint reviews.

### Merge Request Validation Criteria

A Merge Request must:

- ensure the quality and consistency of LOTUSim releases,

- benefit the entire partner ecosystem,

- be sufficiently generic,

- comply with the project’s minimum standards.

In case of disagreement within the Project Team, the decision is escalated to the **Technical Committee**, and then to the **Strategic Committee**, in accordance with the LOTUSim governance process.

## Appendices

### Role Assignment (as of 26/01/2026)

- **Scrum Master**: Malcom NEO

- **Product Owner**: Estelle CHAUVEAU

- **Project Maintainer**: Malcom NEO & Esther RAYSSIGUIE
