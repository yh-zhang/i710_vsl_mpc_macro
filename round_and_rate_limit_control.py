# apply the round and rate limit constraints to the VSL conmmands

def round_and_rate_limit_control(nmpc, t):
	v_temp = nmpc.sim.v[:, [t]]
	# record unconstrained v
	#nmpc.sim.v_act[:, [t]] = v_temp
	
	# round control to nearest multiple of v_pR
	for i in xrange(len(v_temp)):
		v_temp[i] = round(v_temp[i] / nmpc.pre.v_pR) * nmpc.pre.v_pR
	#v_temp = round(v_temp / nmpc.pre.v_pR) * nmpc.pre.v_pR

	# Rate limit
	if t > 0:
		for i in xrange(nmpc.pre.n_u):
			if v_temp[i] <= nmpc.sim.v[i, t-1]:
				nmpc.sim.v[i, t] = max([v_temp[i], nmpc.sim.v[i, t-1] 
					- nmpc.pre.v_pRL, nmpc.pre.v_min])
			else:
				nmpc.sim.v[i, t] = min([v_temp[i], nmpc.sim.v[i, t-1]
				 + nmpc.pre.v_pRL, nmpc.pre.v_max])