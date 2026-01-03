use crate::dsl::types::{ConstraintMode, ScenarioSpec, ScenarioType};
use crate::ltl::formula::{LTLFormula, Proposition};
use anyhow::Result;
use std::collections::HashMap;
use std::sync::Mutex;

/// Trait for scenario-specific LTL generation
pub trait ScenarioPlugin: Send + Sync {
    /// Human-readable name for this scenario type
    fn name(&self) -> &str;

    /// Validate that the scenario spec is compatible with this plugin
    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()>;

    /// Generate the complete LTL formula for this scenario
    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula>;

    /// Generate initial conditions (default implementation provided)
    fn initial_conditions(&self, spec: &ScenarioSpec) -> LTLFormula {
        spec.actors
            .iter()
            .map(|actor| {
                LTLFormula::Atom(Proposition::InLane {
                    actor: actor.id.clone(),
                    lane: actor.lane,
                })
            })
            .reduce(|a, b| a.and(b))
            .unwrap_or(LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 0,
            }))
    }

    /// Generate safety constraints (default implementation using spec constraints)
    fn safety_constraints(&self, spec: &ScenarioSpec) -> LTLFormula {
        let ego = spec.ego();
        let mut constraints = Vec::new();

        for npc in spec.npcs() {
            // TTC constraint
            match spec.constraint_modes.min_ttc() {
                ConstraintMode::Enforce => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::TTCGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            ttc: spec.min_ttc,
                        })
                        .always(),
                    );
                }
                ConstraintMode::Violate => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::TTCGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            ttc: spec.min_ttc,
                        })
                        .negate()
                        .eventually(),
                    );
                }
                ConstraintMode::Ignore => {}
            }

            // Distance constraint
            match spec.constraint_modes.min_distance() {
                ConstraintMode::Enforce => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::DistanceGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            distance: spec.min_distance,
                        })
                        .always(),
                    );
                }
                ConstraintMode::Violate => {
                    constraints.push(
                        LTLFormula::Atom(Proposition::DistanceGT {
                            actor1: ego.id.clone(),
                            actor2: npc.id.clone(),
                            distance: spec.min_distance,
                        })
                        .negate()
                        .eventually(),
                    );
                }
                ConstraintMode::Ignore => {}
            }
        }

        constraints
            .into_iter()
            .reduce(|a, b| a.and(b))
            .unwrap_or(LTLFormula::Atom(Proposition::InLane {
                actor: "ego".to_string(),
                lane: 0,
            }))
    }
}

/// Registry for scenario plugins
pub struct PluginRegistry {
    plugins: HashMap<ScenarioType, Box<dyn ScenarioPlugin>>,
}

impl PluginRegistry {
    pub fn new() -> Self {
        let mut registry = Self {
            plugins: HashMap::new(),
        };

        // Register built-in plugins
        registry.register(ScenarioType::CutInLeft, Box::new(CutInLeftPlugin));
        registry.register(ScenarioType::CutInRight, Box::new(CutInRightPlugin));
        registry.register(ScenarioType::Following, Box::new(FollowingPlugin));

        registry
    }

    pub fn register(&mut self, scenario_type: ScenarioType, plugin: Box<dyn ScenarioPlugin>) {
        self.plugins.insert(scenario_type, plugin);
    }

    pub fn generate(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let plugin = self
            .plugins
            .get(&spec.scenario_type)
            .ok_or_else(|| anyhow::anyhow!("No plugin registered for {:?}", spec.scenario_type))?;

        plugin.validate_spec(spec)?;
        plugin.generate_ltl(spec)
    }
}

// Global registry with lazy initialization
lazy_static::lazy_static! {
    pub static ref REGISTRY: Mutex<PluginRegistry> = Mutex::new(PluginRegistry::new());
}

// Plugin implementation for CutInLeft scenario
struct CutInLeftPlugin;

impl ScenarioPlugin for CutInLeftPlugin {
    fn name(&self) -> &str {
        "Cut-In Left"
    }

    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
        let npc_count = spec.npcs().count();
        if npc_count != 1 {
            anyhow::bail!(
                "Cut-in left requires exactly 1 NPC, found {}",
                npc_count
            );
        }

        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        if ego.lane != 1 {
            anyhow::bail!(
                "Cut-in left requires ego in lane 1, found lane {}",
                ego.lane
            );
        }
        if npc.lane != 0 {
            anyhow::bail!(
                "Cut-in left requires NPC in lane 0, found lane {}",
                npc.lane
            );
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        // Initial conditions
        let init = self
            .initial_conditions(spec)
            .and(LTLFormula::Atom(Proposition::Ahead {
                actor1: npc.id.clone(),
                actor2: ego.id.clone(),
            }));

        // Behavior: NPC eventually changes to lane 1
        let behavior = LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: 1,
        })
        .eventually()
        .and(
            LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 0,
            })
            .until(LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 1,
            })),
        );

        // Safety constraints
        let safety = self.safety_constraints(spec);

        Ok(init.and(behavior).and(safety))
    }
}

// Plugin implementation for CutInRight scenario
struct CutInRightPlugin;

impl ScenarioPlugin for CutInRightPlugin {
    fn name(&self) -> &str {
        "Cut-In Right"
    }

    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
        let npc_count = spec.npcs().count();
        if npc_count != 1 {
            anyhow::bail!(
                "Cut-in right requires exactly 1 NPC, found {}",
                npc_count
            );
        }

        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        if ego.lane != 0 {
            anyhow::bail!(
                "Cut-in right requires ego in lane 0, found lane {}",
                ego.lane
            );
        }
        if npc.lane != 1 {
            anyhow::bail!(
                "Cut-in right requires NPC in lane 1, found lane {}",
                npc.lane
            );
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        // Initial conditions
        let init = self
            .initial_conditions(spec)
            .and(LTLFormula::Atom(Proposition::Ahead {
                actor1: npc.id.clone(),
                actor2: ego.id.clone(),
            }));

        // Behavior: NPC eventually changes to lane 0
        let behavior = LTLFormula::Atom(Proposition::InLane {
            actor: npc.id.clone(),
            lane: 0,
        })
        .eventually()
        .and(
            LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 1,
            })
            .until(LTLFormula::Atom(Proposition::InLane {
                actor: npc.id.clone(),
                lane: 0,
            })),
        );

        // Safety constraints
        let safety = self.safety_constraints(spec);

        Ok(init.and(behavior).and(safety))
    }
}

// Plugin implementation for Following scenario
struct FollowingPlugin;

impl ScenarioPlugin for FollowingPlugin {
    fn name(&self) -> &str {
        "Following"
    }

    fn validate_spec(&self, spec: &ScenarioSpec) -> Result<()> {
        let npc_count = spec.npcs().count();
        if npc_count != 1 {
            anyhow::bail!("Following requires exactly 1 NPC, found {}", npc_count);
        }

        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        if ego.lane != npc.lane {
            anyhow::bail!(
                "Following requires both actors in same lane: ego={}, npc={}",
                ego.lane,
                npc.lane
            );
        }

        Ok(())
    }

    fn generate_ltl(&self, spec: &ScenarioSpec) -> Result<LTLFormula> {
        let ego = spec.ego();
        let npc = spec.npcs().next().unwrap();

        // Initial conditions: both in same lane, NPC ahead
        let init = self
            .initial_conditions(spec)
            .and(LTLFormula::Atom(Proposition::Ahead {
                actor1: npc.id.clone(),
                actor2: ego.id.clone(),
            }));

        // Behavior: Always in same lane, maintain distance
        let behavior = LTLFormula::Atom(Proposition::SameLane {
            actor1: ego.id.clone(),
            actor2: npc.id.clone(),
        })
        .always()
        .and(LTLFormula::Atom(Proposition::Ahead {
            actor1: npc.id.clone(),
            actor2: ego.id.clone(),
        }).always());

        // Safety constraints
        let safety = self.safety_constraints(spec);

        Ok(init.and(behavior).and(safety))
    }
}
