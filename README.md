# Digital Control 2026

Course work and robot experiments, organized by week.

| Directory | Contents |
| --- | --- |
| [`week01/`](week01/) | DC motor model and parameter script |
| [`week02/`](week02/) | Motor identification, controller exercises, and notebooks |
| [`week03/`](week03/) | Difference-equation stability exercise |
| [`week04/`](week04/) | Discrete-time response and pole/zero exercises |
| [`week05/`](week05/) | Sampled P-controller model and experiments |
| [`week06/`](week06/) | Robot models, motor-log analysis, and simulations |
| [`src/`](src/) | Robot firmware and archived firmware bundle |
| [`documentation/`](documentation/) | Lecture material and design notes |
| [`tests/`](tests/) | Host-side firmware checks |

Python dependencies and notebook tooling are defined by `pyproject.toml` and
`uv.lock` at the repository root. MATLAB/Simulink scripts are kept with the
week they belong to. Weeks 05 and 06 include detailed run guides; earlier
weeks contain their exercise notes and scripts.
