function world = ap_world_init(scn, cfg)









ap_rng(cfg.sim.seed);

world.t      = 0;
world.agents = scn.agents;
world.rp     = scn.rp;
world.goal   = scn.goal;
world.name   = scn.name;

world.ego       = scn.ego;
world.ego.delta = 0;
world.ego.a     = 0;

world.collided = false;
world.reached  = false;
end
