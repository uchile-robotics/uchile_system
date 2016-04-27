/*
 * LegDetector.cpp
 *
 *  Created on: 14-10-2014
 *      Author: matias.pavez.b@gmail.com
 */

#include <bender_laser/LegDetector.hpp>

/**
 * Work based on the paper:
 * [1] Nicola Bellotto and Huosheng Hu, “Multisensor-Based Human Detection and Tracking for
 * Mobile Service Robots”, IEEE Transactions on Systems, Man and Cybernetics, February, 2009
 */

/**
 * TODO List:
 * - tuneo
 * - merge de edges parecidos
 */
namespace bender_laser {

LegDetector::LegDetector() {

	ros::NodeHandle priv("~");

	// parameters
	bender_config::ParameterServerWrapper psw;
	// - - Note: using the configuration shown in [1] - -
	psw.getParameter("leg_model/a_min", _leg_model.a_min, 0.07); // [m] min leg width
	psw.getParameter("leg_model/a_max", _leg_model.a_max, 0.18); // [m] max leg width
	psw.getParameter("leg_model/b_max", _leg_model.b_max, 0.45); // [m] max step length
	psw.getParameter("leg_model/c_min", _leg_model.c_min, 0.12); // [m] min width for SL
	psw.getParameter("leg_model/c_max", _leg_model.c_max, 0.40); // [m] max width for SL

	// - -                                            - -
	psw.getParameter("scan_edge_threshold", _edge_threshold, 0.15); // [m] min distance for edge detection
	psw.getParameter("local_minimization_window_size", _local_minimization_window_size, 5);
	psw.getParameter("local_maximization_window_size", _local_maximization_window_size, 5);
	psw.getParameter("tf_frame_out", _frame_out, "/bender/base_link");
	// XXX: aumentar delta --> problemas para detectar patrones??

	// - - - - - - - - - - - - - - - -  P U B L I S H E R S - - - - - - - - - - - - - - - -
	_detection_markers_pub = priv.advertise<visualization_msgs::MarkerArray>("detection_markers",1, this);
	_local_minimization_pub = priv.advertise<sensor_msgs::LaserScan>("local_minimization",1, this);
	_local_maximization_pub = priv.advertise<sensor_msgs::LaserScan>("local_maximization",1, this);
	_edge_detection_marker_pub = priv.advertise<visualization_msgs::Marker>("edge_detections",1, this);
	_detection_pub = priv.advertise<bender_msgs::PoseDetections>("leg_detections",1, this);

	// - - - - - - - - - - - - - - - - S U B S C R I B E R S - - - - - - - - - - - - - - -
	_laser_scan_sub = priv.subscribe("scan", 1, &LegDetector::callback_laserScan,this);

	// [important!] wait for timer initialization!
	while (ros::ok() && ros::Time::now().isZero());

	ros::Duration(1.0).sleep();

	ROS_INFO("Ready to Work");
}

LegDetector::~LegDetector() {}


void LegDetector::transform_detections(const std::vector<LegDetection>& in, std::vector<LegDetection>& out) {

	if (in.empty()) {
		return;
	}

	std::string frame_in = in[0].body.header.frame_id;
	ros::Time detection_time = in[0].body.header.stamp;

	out.clear();
	LegDetection detection;
	try {
		_tf_listener.waitForTransform(_frame_out, frame_in, detection_time, ros::Duration(2.0));

		std::vector<LegDetection>::const_iterator it;
		for (it = in.begin(); it != in.end(); ++it) {

			detection.pattern = it->pattern;
			_tf_listener.transformPose(_frame_out, it->body, detection.body);
			out.push_back(detection);
		}

	} catch (tf::ConnectivityException &e) {
		ROS_WARN_STREAM("Requested a transform between unconnected trees!: " << e.what());
		out.clear();
		return;

	} catch (tf::TransformException &e) {
		ROS_WARN_STREAM("Transform Exception: " << e.what());
		out.clear();
		return;
	}
}


void LegDetector::leg_publisher(const std::vector<LegDetection>& leg_detections) {

	std::vector<LegDetection> tf_detections;
	transform_detections(leg_detections, tf_detections);

	// publish detections
	bender_msgs::PoseDetections msg;
	for (int k=0; k<tf_detections.size(); ++k) {

		LegDetection detection = tf_detections[k];
		geometry_msgs::Pose single_detection;

		single_detection.position = detection.body.pose.position;
		single_detection.orientation = detection.body.pose.orientation;

		std::string info = LegsPattern::toString(detection.pattern.type);

		msg.detections.push_back(single_detection);
		msg.info.push_back(info);
	}
	if ( tf_detections.size() > 0 ) {
		msg.header = tf_detections[0].body.header;
		_detection_pub.publish(msg);
	}


	if (_detection_markers_pub.getNumSubscribers() > 0) {

		// visualization step
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		visualization_msgs::MarkerArray marker_array;
		visualization_msgs::Marker marker;

		marker.ns = "leg_detection";
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::MODIFY;
		marker.lifetime = ros::Duration(0.1);

		// size
		marker.scale.x = 0.15;
		marker.scale.y = 0.15;
		marker.scale.z = 0.80;

		// colors
		std_msgs::ColorRGBA LAcolor, FScolor, SLcolor;
		LAcolor.r = 1.0; LAcolor.g = 1.0; LAcolor.b = 0.0; LAcolor.a = 1.0;
		FScolor.r = 0.0; FScolor.g = 1.0; FScolor.b = 1.0; FScolor.a = 1.0;
		SLcolor.r = 1.0; SLcolor.g = 0.0; SLcolor.b = 1.0; SLcolor.a = 1.0;

		for (int k=0; k<tf_detections.size(); ++k) {

			LegDetection detection = tf_detections[k];
			marker.header = detection.body.header;
			marker.pose = detection.body.pose;

			// set color
			switch (detection.pattern.type) {

				case LegsPattern::LA: { marker.color = LAcolor; break;}
				case LegsPattern::FS: { marker.color = FScolor; break;}
				case LegsPattern::SL: { marker.color = SLcolor; break;}
				default: {
					marker.color.a = marker.color.r = marker.color.g = marker.color.b = 1;
				}
			}

			// add person
			marker.id = k;
			marker_array.markers.push_back(marker);
		}

		// publish
		_detection_markers_pub.publish(marker_array);
	}
}

/**
 * TODO: PUEDE SER OPTIMIZADO!!
 * - Se calculan muchos mínimos, más de los necesarios
 */
void LegDetector::local_minimization(const sensor_msgs::LaserScan &input, sensor_msgs::LaserScan &output) {

	// copy scan header
	output.header.frame_id = input.header.frame_id;
	output.header.stamp = ros::Time::now();

	output.angle_min = input.angle_min;
	output.angle_max = input.angle_max;
	output.angle_increment = input.angle_increment;

	output.time_increment = input.time_increment;
	output.scan_time = input.scan_time;

	output.range_min = input.range_min;
	output.range_max = input.range_max;

	// - - -  process  - - -

	// work with 'delta' points/side --> (2*delta + 1)[points]
	int delta = floor(  (_local_minimization_window_size-1.0)/2  ) ;

	for (int i=0; i<input.ranges.size(); ++i) {

		int start_idx = std::max(0,i-delta);
		int end_idx = std::min((int)input.ranges.size()-1,i+delta);
		//ROS_WARN_STREAM("start_idx=" << start_idx << ", end_idx=" << end_idx);

		if (input.ranges[i] == input.range_max) {
			output.ranges.push_back(input.range_max);
			continue;
		}

		float min_value = input.range_max;
		for (int k=start_idx; k<=end_idx; ++k) {

			//ROS_WARN_STREAM("Hello A");

			if (input.ranges[k] < min_value && input.ranges[k] != input.range_max) {
				//ROS_WARN_STREAM("Hello MIN");
				min_value = input.ranges[k];
			}
		}
		output.ranges.push_back(min_value);
	}

}

/**
 * TODO: PUEDE SER OPTIMIZADO!!
 * - Se calculan muchos máximos, más de los necesarios
 */
void LegDetector::local_maximization(const sensor_msgs::LaserScan &input, sensor_msgs::LaserScan &output) {

	// copy scan header
	output.header.frame_id = input.header.frame_id;
	output.header.stamp = ros::Time::now();

	output.angle_min = input.angle_min;
	output.angle_max = input.angle_max;
	output.angle_increment = input.angle_increment;

	output.time_increment = input.time_increment;
	output.scan_time = input.scan_time;

	output.range_min = input.range_min;
	output.range_max = input.range_max;

	// - - -  process  - - -

	// work with 'delta' points/side --> (2*delta + 1)[points]
	int delta = floor(  (_local_maximization_window_size-1.0)/2  ) ;

	for (int i=0; i<input.ranges.size(); ++i) {

		int start_idx = std::max(0,i-delta);
		int end_idx = std::min((int)input.ranges.size()-1,i+delta);
		//ROS_WARN_STREAM("start_idx=" << start_idx << ", end_idx=" << end_idx);

		if (input.ranges[i] == input.range_max) {
			output.ranges.push_back(input.range_max);
			continue;
		}

		float max_value = input.range_min;
		for (int k=start_idx; k<=end_idx; ++k) {

			//ROS_WARN_STREAM("Hello A");

			if (input.ranges[k] > max_value && input.ranges[k] != input.range_max) {
				//ROS_WARN_STREAM("Hello MIN");
				max_value = input.ranges[k];
			}
		}
		output.ranges.push_back(max_value);
	}
}


/**
 * TODO: juntar bordes que estan muy alineados y demasiado cercanos
 * -> importante al disminuir el threshold!, menor th --> más bordes, que pueden corresponder al mismo
 * --> se necesita merge
 */
void LegDetector::edge_detection(const sensor_msgs::LaserScan &scan, std::vector<Edge> &edges) {

	// prepare visualization marker
	visualization_msgs::Marker marker;
	marker.header.frame_id = scan.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "edge_detection";
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.pose.orientation.w = 1;
	marker.id = 0;
	std_msgs::ColorRGBA color_left;
	color_left.a = 1.0;
	color_left.g = 1.0;
	std_msgs::ColorRGBA color_right;
	color_right.a = 1.0;
	color_right.r = 1.0;
	marker.scale.x = 0.02; // width of line segment

	for (int k=0; k<scan.ranges.size()-1; ++k) {

		float r1 = scan.ranges[k];
		float r2 = scan.ranges[k+1];
		float diff = r2-r1;

		if ( fabs(diff) > _edge_threshold) {

			Edge edge;
			edge.r1 = r1;
			edge.r2 = r2;
			edge.theta1 = scan.angle_min +   k   * scan.angle_increment;
			edge.theta2 = scan.angle_min + (k+1) * scan.angle_increment;

			// point 1 & point 2
			geometry_msgs::Point p1, p2;
			p1.x = r1*cosf(edge.theta1);
			p1.y = r1*sinf(edge.theta1);
			p2.x = r2*cosf(edge.theta2);
			p2.y = r2*sinf(edge.theta2);
			marker.points.push_back(p1);
			marker.points.push_back(p2);

			// right or left edge
			if (diff > 0) {
				edge.type = Edge::RIGHT_EDGE;
				marker.colors.push_back(color_right); // p1 color
				marker.colors.push_back(color_right); // p2 color

			} else {
				edge.type = Edge::LEFT_EDGE;
				marker.colors.push_back(color_left); // p1 color
				marker.colors.push_back(color_left); // p2 color
			}
			edges.push_back(edge);
		}
	}

	// TODO: merge adjacent edges

	_edge_detection_marker_pub.publish(marker);
}

void LegDetector::extract_LA_Patterns(std::vector<Edge> &edges, std::string frame_id, std::vector<LegDetection> &leg_detections) {

	// OBS: recordar tomar en cuenta el caso {L,L,R}, en caso de que el primer "L" esté al final!
	// y tomar en cuenta los otros casos patológicos, EJ: {L,R}, cuando L está al final, y R está al inicio
	// Extract the set P_LA of LA patterns from E

	Edge em0, em1;
	//ROS_WARN_STREAM("("<< edges.size() <<") edges");

	int m = 0;
	while (m < edges.size() ) {

		int idx_m0 = m;
		int idx_m1 = (m+1)%edges.size();

		// seek m0 as Left Edge := {L}
		em0 = edges[idx_m0];
		if (em0.type != Edge::LEFT_EDGE) { m++; continue; }

		// and seek m1 as Right Edge := {R}
		em1 = edges[idx_m1];
		if (em1.type != Edge::RIGHT_EDGE){ m++; continue; }

		// check width constraint for pair {L,R}
		float r_mean = (em0.r2 + em1.r1)/2.0;
		float dtheta = em1.theta1 - em0.theta2;
		float a = 2.0*r_mean*sinf(dtheta/2.0);

		//ROS_WARN_STREAM("a=" << a);

		// check {L,R} and leg width constraints
		if ( _leg_model.a_min <= a && a <= _leg_model.a_max ) {

			//ROS_WARN_STREAM("I found the first {L,R} pattern");

			bool found = false;
			Edge en0, en1;
			int idx_n0 = (idx_m0+2-1)%edges.size();
			int cnt = 0;
			while ( !found && cnt < edges.size()-2) {

				idx_n0 = (idx_n0+1)%edges.size();
				cnt++;

				int idx_n1 = (idx_n0+1)%edges.size();

				// seek n0 as Left Edge := {L}
				en0 = edges[idx_n0];
				if (en0.type != Edge::LEFT_EDGE) continue;

				// and seek n1 as Right Edge := {R}
				en1 = edges[idx_n1];
				if (en1.type != Edge::RIGHT_EDGE) continue;

				// check width constraint for pair {L,R}
				r_mean = (en0.r2 + en1.r1)/2.0;
				dtheta = en1.theta1 - en0.theta2;
				a = 2.0*r_mean*sinf(dtheta/2.0);

				// check {L,R} and leg width constraints
				if ( _leg_model.a_min <= a && a <= _leg_model.a_max ) {

					//ROS_WARN_STREAM("I found a second {L,R} pattern");

					// check {L,R,L,R} LA constraint
					float m_r = (em0.r2 + em1.r1)/2;
					float m_theta = (em0.theta2 + em1.theta1)/2.0;
					float n_r = (en0.r2 + en1.r1)/2;
					float n_theta = (en0.theta2 + en1.theta1)/2.0;
					float b = sqrtf(m_r*m_r + n_r*n_r - 2*m_r*n_r*cosf(n_theta-m_theta));

					if (b < _leg_model.b_max) {

						// add detection
						LegDetection detection;
						detection.body.header.frame_id = frame_id;
						detection.body.header.stamp = ros::Time::now();
						detection.pattern = LA_Pattern(em0, em1, en0, en1);

						// pose of the detected leg is calculated from the midpoint of the pattern
						float center_r = (m_r + n_r)/2.0;
						float center_theta = (m_theta + n_theta)/2.0;
						detection.body.pose.position.x = center_r*cosf(center_theta);
						detection.body.pose.position.y = center_r*sinf(center_theta);
						detection.body.pose.orientation.w = 1.0;
						leg_detections.push_back(detection);

						// remove edges (container will be resized!, so elements must be deleted in order)
						int idxs_array[] = {idx_m0, idx_m1, idx_n0, idx_n1};
						std::vector<int> to_erase_idxs (idxs_array, idxs_array + sizeof(idxs_array) / sizeof(int));
						std::sort (to_erase_idxs.begin(), to_erase_idxs.end());

						//ROS_WARN_STREAM("indexes: " << to_erase_idxs[3] << "," << to_erase_idxs[2] << "," << to_erase_idxs[1] << "," << to_erase_idxs[0]);
						edges.erase(edges.begin() + to_erase_idxs[3]);
						edges.erase(edges.begin() + to_erase_idxs[2]);
						edges.erase(edges.begin() + to_erase_idxs[1]);
						edges.erase(edges.begin() + to_erase_idxs[0]);
						found = true;
						break;
					} // end if (b constraint)
				} // end if (a constraint for n)
			} // end while n0 n1
		} // end if (a constraint for m)
		m++;
	} // end while m0, m1
}

void LegDetector::extract_FS_Patterns(std::vector<Edge> &edges, std::string frame_id, std::vector<LegDetection> &leg_detections) {

	Edge e_l, e_m, e_r;

	int m = 0;
	while (m < edges.size() ) {

		int idx_l = m;
		int idx_m = (m+1)%edges.size();
		int idx_r = (m+2)%edges.size();

		// seek l, m, r as {L, L, R} || {L, R, R}
		e_l = edges[idx_l]; // left edge
		if (e_l.type != Edge::LEFT_EDGE) { m++; continue; }

		e_r = edges[idx_r]; // right edge
		if (e_r.type != Edge::RIGHT_EDGE){ m++; continue; }

		e_m = edges[idx_m]; // middle edge (L or R)

		//ROS_WARN_STREAM("I found a {L,*,R} pattern");

		// check width constraint for triplet {L,X,R}
		float l_r = (e_l.r2 + e_m.r1)/2.0;
		float l_dtheta = e_m.theta1 - e_l.theta2;
		float l_a = 2.0*l_r*sinf(l_dtheta/2.0);

		float r_r = (e_r.r1 + e_m.r2)/2.0;
		float r_dtheta = e_r.theta1 - e_m.theta2;
		float r_a = 2.0*r_r*sinf(r_dtheta/2.0);

		float b = sqrtf(l_r*l_r + r_r*r_r - 2*l_r*r_r*cosf(l_dtheta-r_dtheta));

		//ROS_WARN_STREAM("left_a=" << l_a << ", right a=" << r_a << ", b=" << b);

		// check triplet constraints
		if (   _leg_model.a_min <= l_a && l_a <= _leg_model.a_max
			&& _leg_model.a_min <= r_a && r_a <= _leg_model.a_max
			&& b <= _leg_model.b_max
		) {

			// add detection
			LegDetection detection;
			detection.body.header.frame_id = frame_id;
			detection.body.header.stamp = ros::Time::now();
			detection.pattern = FS_Pattern(e_l, e_m, e_r);

			// pose of the detected leg is calculated from the midpoint of the pattern
			float center_r = (l_r + r_r)/2.0;
			float center_theta = ( (e_m.theta1 + e_l.theta2) + (e_r.theta1 + e_m.theta2))/4.0;
			detection.body.pose.position.x = center_r*cosf(center_theta);
			detection.body.pose.position.y = center_r*sinf(center_theta);
			detection.body.pose.orientation.w = 1.0;
			leg_detections.push_back(detection);

			// remove edges (container will be resized!, so elements must be deleted in order)
			int idxs_array[] = {idx_l, idx_m, idx_r};
			std::vector<int> to_erase_idxs (idxs_array, idxs_array + sizeof(idxs_array) / sizeof(int));
			std::sort (to_erase_idxs.begin(), to_erase_idxs.end());

			//ROS_WARN_STREAM("indexes: " << to_erase_idxs[2] << "," << to_erase_idxs[1] << "," << to_erase_idxs[0]);
			edges.erase(edges.begin() + to_erase_idxs[2]);
			edges.erase(edges.begin() + to_erase_idxs[1]);
			edges.erase(edges.begin() + to_erase_idxs[0]);
		} // end if (constraints)
		m++;
	} // end while
}

void LegDetector::extract_SL_Patterns(std::vector<Edge> &edges, std::string frame_id, std::vector<LegDetection> &leg_detections) {

	Edge e_l, e_r;

	int m = 0;
	while (m < edges.size() ) {

		int idx_l = m;
		int idx_r = (m+1)%edges.size();

		// seek l, r as {L, R}
		e_l = edges[idx_l]; // left edge
		if (e_l.type != Edge::LEFT_EDGE) { m++; continue; }

		e_r = edges[idx_r]; // right edge
		if (e_r.type != Edge::RIGHT_EDGE){ m++; continue; }

		//ROS_WARN_STREAM("I found a {L, R} pattern");

		// check width constraint for triplet {L,X,R}
		float r = (e_l.r2 + e_r.r1)/2.0;
		float dtheta = e_r.theta1 - e_l.theta2;
		float c = 2.0*r*sinf(dtheta/2.0);

		//ROS_WARN_STREAM("c=" << c);

		// check triplet constraints
		if ( _leg_model.c_min <= c && c <= _leg_model.c_max ) {

			// add detection
			LegDetection detection;
			detection.body.header.frame_id = frame_id;
			detection.body.header.stamp = ros::Time::now();
			detection.pattern = SL_Pattern(e_l, e_r);

			// pose of the detected leg is calculated from the midpoint of the pattern
			float center_r = r;
			float center_theta = (e_r.theta1 + e_l.theta2)/2;
			detection.body.pose.position.x = center_r*cosf(center_theta);
			detection.body.pose.position.y = center_r*sinf(center_theta);
			detection.body.pose.orientation.w = 1.0;
			leg_detections.push_back(detection);

			// remove edges (container will be resized!, so elements must be deleted in order)
			int idxs_array[] = {idx_l, idx_r};
			std::vector<int> to_erase_idxs (idxs_array, idxs_array + sizeof(idxs_array) / sizeof(int));
			std::sort (to_erase_idxs.begin(), to_erase_idxs.end());

			//ROS_WARN_STREAM("indexes: " << to_erase_idxs[1] << "," << to_erase_idxs[0]);
			edges.erase(edges.begin() + to_erase_idxs[1]);
			edges.erase(edges.begin() + to_erase_idxs[0]);
		} // end if (constraints)
		m++;
	} // end while
}

void LegDetector::callback_laserScan(const sensor_msgs::LaserScan &scan) {

	//ROS_WARN_STREAM("Received (" << scan.ranges.size() << ") points.");

	// local minimization
	sensor_msgs::LaserScan minimized;
	local_minimization(scan, minimized);
	_local_minimization_pub.publish(minimized);

	// local maximization
	sensor_msgs::LaserScan maximized;
	local_minimization(minimized, maximized);
	_local_maximization_pub.publish(maximized);

	// edge detection
	std::vector<Edge> edges;
	edge_detection(maximized, edges);

	// pattern extraction
	std::vector<LegDetection> leg_detections;
	extract_LA_Patterns(edges, scan.header.frame_id, leg_detections);
	extract_FS_Patterns(edges, scan.header.frame_id, leg_detections);
	extract_SL_Patterns(edges, scan.header.frame_id, leg_detections);

	// visualize results
	leg_publisher(leg_detections);
}

} /* namespace bender_laser */

int main(int argc, char **argv){

	ros::init(argc, argv, "leg_detector");

	boost::scoped_ptr<bender_laser::LegDetector> node(
				new bender_laser::LegDetector()
	);

	ros::spin();
	printf("\nQuitting... \n\n");

	return 0;
}
