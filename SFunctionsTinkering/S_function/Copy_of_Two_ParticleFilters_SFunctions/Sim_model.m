function X_sim_next = Sim_model(sim,u0,x0)
sim.set('x',x0);
sim.set('u', u0);
sim.solve();
X_sim_next = sim.get('xn');
end