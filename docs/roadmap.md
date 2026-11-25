# Roadmap

1. **PCL source → PCLgraph automation**: push the conversion pipeline to maximum automation—derive node declarations, port/property metadata, point-type instantiations, registration, and tests from PCL source and headers instead of hand-writing each node.
2. **DLL generation**: export a graph as a reusable DLL/library with a generated entry point, similar to Halcon's HDEVELOP workflow.
3. **Agent-generated pipelines**: let an agent turn a common point-cloud task into a valid graph (project JSON) of existing PCLgraph nodes, backed by type checks, sample data, and metrics so the generated combination can be verified. This needs a headless graph runner and metric nodes so the agent can execute and validate what it generates.

## Open decisions

- **Distribution model**: whether custom nodes ship as a separate plugin library, PCLgraph becomes a desktop application, or it stays a library like taskgraph is still undecided. Install/export and packaging follow that decision.
