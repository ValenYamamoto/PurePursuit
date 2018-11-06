
public class Splines {
	
	Coord pf_spline_coords(Spline s, double percentage) {
	    percentage = Math.max(Math.min(percentage, 1), 0);
	    double x = percentage * s.knot_distance;
	    double y = (s.a*x + s.b) * (x*x*x*x) + (s.c*x + s.d) * (x*x) + s.e*x;    // Heh, sex
	    
	    double cos_theta = Math.cos(s.angle_offset);
	    double sin_theta = Math.sin(s.angle_offset);
	    
	    Coord c = new Coord(
	        x * cos_theta - y * sin_theta + s.x_offset,
	        x * sin_theta + y * cos_theta + s.y_offset
	    );
	    return c;
	}

	double pf_spline_deriv(Spline s, double percentage) {
	    double x = percentage * s.knot_distance;
	    return (5*s.a*x + 4*s.b) * (x*x*x) + (3*s.c*x + 2*s.d) * x + s.e;
	}

	double pf_spline_deriv_2(double a, double b, double c, double d, double e, double k, double p) {
	    double x = p * k;
	    return (5*a*x + 4*b) * (x*x*x) + (3*c*x + 2*d) * x + e;
	}

	double pf_spline_angle(Spline s, double percentage) {
	    return bound_radians(Math.atan(pf_spline_deriv(s, percentage)) + s.angle_offset);
	}

	double pf_spline_distance(Spline s, int sample_count) {
	    double sample_count_d = (double) sample_count;
	    
	    double a = s->a; double b = s->b; double c = s->c; 
	    double d = s->d; double e = s->e; double knot = s->knot_distance;
	    
	    double arc_length = 0, t = 0, dydt = 0;
	    
	    double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);
	    
	    double integrand = 0;
	    double last_integrand = sqrt(1 + deriv0*deriv0) / sample_count_d;
	    
	    int i;
	    for (i = 0; i <= sample_count; i = i + 1) {
	        t = i / sample_count_d;
	        dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
	        integrand = sqrt(1 + dydt*dydt) / sample_count_d;
	        arc_length += (integrand + last_integrand) / 2;
	        last_integrand = integrand;
	    }
	    double al = knot * arc_length;
	    s->arc_length = al;
	    return al;
	}

	double pf_spline_progress_for_distance(Spline s, double distance, int sample_count) {
	    double sample_count_d = (double) sample_count;
	    
	    double a = s.a; double b = s.b; double c = s.c;
	    double d = s.d; double e = s.e; double knot = s.knot_distance;
	    
	    double arc_length = 0, t = 0, dydt = 0, last_arc_length = 0;
	    
	    double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);

	    double integrand = 0;
	    double last_integrand = sqrt(1 + deriv0*deriv0) / sample_count_d;
	    
	    distance /= knot;
	    
	    int i;
	    for (i = 0; i <= sample_count; i = i + 1) {
	        t = i / sample_count_d;
	        dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
	        integrand = sqrt(1 + dydt*dydt) / sample_count_d;
	        arc_length += (integrand + last_integrand) / 2;
	        if (arc_length > distance) break;
	        last_integrand = integrand;
	        last_arc_length = arc_length;
	    }
	    
	    double interpolated = t;
	    if (arc_length != last_arc_length) {
	        interpolated += ((distance - last_arc_length)
	            / (arc_length - last_arc_length) - 1) / sample_count_d;
	    }
	    return interpolated;
	}
	
	TrajectoryCandidate cand_LV;

	int pathfinder_prepare(const Waypoint *path, int path_length, void (*fit)(Waypoint,Waypoint,Spline*), int sample_count, double dt,
	        double max_velocity, double max_acceleration, double max_jerk, TrajectoryCandidate *cand) {
	    if (path_length < 2) return -1;
	    
	    cand->saptr = (Spline *)malloc((path_length - 1) * sizeof(Spline));
	    cand->laptr = (double *)malloc((path_length - 1) * sizeof(double));
	    double totalLength = 0;
	    
	    int i;
	    for (i = 0; i < path_length-1; i++) {
	        Spline s;
	        fit(path[i], path[i+1], &s);
	        double dist = pf_spline_distance(&s, sample_count);
	        cand->saptr[i] = s;
	        cand->laptr[i] = dist;
	        totalLength += dist;
	    }
	    
	    TrajectoryConfig config = {dt, max_velocity, max_acceleration, max_jerk, 0, path[0].angle,
	        totalLength, 0, path[0].angle, sample_count};
	    TrajectoryInfo info = pf_trajectory_prepare(config);
	    int trajectory_length = info.length;
	    
	    cand->totalLength = totalLength;
	    cand->length = trajectory_length;
	    cand->path_length = path_length;
	    cand->info = info;
	    cand->config = config;
	    
	    return trajectory_length;
	}
	
	int pathfinder_generate(TrajectoryCandidate *c, Segment *segments) {
	    int trajectory_length = c->length;
	    int path_length = c->path_length;
	    double totalLength = c->totalLength;
	    
	    Spline *splines = (c->saptr);
	    double *splineLengths = (c->laptr);
	    
	    int trajectory_status = pf_trajectory_create(c->info, c->config, segments);
	    if (trajectory_status < 0) return trajectory_status;
	    
	    int spline_i = 0;
	    double spline_pos_initial = 0, splines_complete = 0;
	    
	    int i;
	    for (i = 0; i < trajectory_length; ++i) {
	        double pos = segments[i].position;

	        int found = 0;
	        while (!found) {
	            double pos_relative = pos - spline_pos_initial;
	            if (pos_relative <= splineLengths[spline_i]) {
	                Spline si = splines[spline_i];
	                double percentage = pf_spline_progress_for_distance(si, pos_relative, c->config.sample_count);
	                Coord coords = pf_spline_coords(si, percentage);
	                segments[i].heading = pf_spline_angle(si, percentage);
	                segments[i].x = coords.x;
	                segments[i].y = coords.y;
	                found = 1;
	            } else if (spline_i < path_length - 2) {
	                splines_complete += splineLengths[spline_i];
	                spline_pos_initial = splines_complete;
	                spline_i += 1;
	            } else {
	                Spline si = splines[path_length - 2];
	                segments[i].heading = pf_spline_angle(si, 1.0);
	                Coord coords = pf_spline_coords(si, 1.0);
	                segments[i].x = coords.x;
	                segments[i].y = coords.y;
	                found = 1;
	            }
	        }
	    }

	    
	    return trajectory_length;
	}
}

public static class Coord {
	private double x, y;
	
	public Coord() {}
	
	public Coord(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public void setY(double y) {
		this.y = y;
	}
}

public static class Spline {
	 double a, b, c, d, e;
	 double x_offset, y_offset, angle_offset, knot_distance, arc_length;
}

