function means_gpmp2 = optimize_gpmp2(input_dataset, nt)

   import gtsam.*
   import gpmp2.*

    rows = input_dataset.rows;
    cols = input_dataset.cols;
    cell_size = input_dataset.cell_size;
    origin_point2 = Point2(input_dataset.origin_x, input_dataset.origin_y);

    % signed distance field
    field = signedDistanceField2D(input_dataset.map, cell_size);
    sdf = PlanarSDF(origin_point2, cell_size, field);

    % settings
    total_time_sec = 4.0;
    total_check_step = 40;
    delta_t = total_time_sec / nt;
    check_inter = total_check_step / nt - 1;

    % use GP interpolation
    use_GP_inter = false;

    % arm model
    arm = generateArm('SimpleTwoLinksArm');

    % GP
    Qc = eye(2);
    Qc_model = noiseModel.Gaussian.Covariance(Qc); 

    % Obstacle avoid settings
    cost_sigma = 0.05;
    epsilon_dist = 0.1;

    % prior to start/goal
    pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
    vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

    % start and end conf
    start_conf = [0, 0]';
    start_vel = [0, 0]';
    end_conf = [pi/2, 0]';
    end_vel = [0, 0]';
    avg_vel = (end_conf / nt) / delta_t;

    % plot param
    pause_time = total_time_sec / nt;

    % ------------------------- init optimization ------------------------- 
    graph = NonlinearFactorGraph;
    init_values = Values;

    for i = 0 : nt
        key_pos = symbol('x', i);
        key_vel = symbol('v', i);

        % initialize as straight line in conf space
        pose = start_conf * (nt-i)/nt + end_conf * i/nt;
        vel = avg_vel;
        init_values.insert(key_pos, pose);
        init_values.insert(key_vel, vel);

        % start/end priors
        if i==0
            graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
            graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
        elseif i==nt
            graph.add(PriorFactorVector(key_pos, end_conf, pose_fix));
            graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
        end

        % GP priors and cost factor
        if i > 0
            key_pos1 = symbol('x', i-1);
            key_pos2 = symbol('x', i);
            key_vel1 = symbol('v', i-1);
            key_vel2 = symbol('v', i);
            graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
                key_pos2, key_vel2, delta_t, Qc_model));

            % cost factor
            graph.add(ObstaclePlanarSDFFactorArm(...
                key_pos, arm, sdf, cost_sigma, epsilon_dist));

            % GP cost factor
            if use_GP_inter & check_inter > 0
                for j = 1:check_inter
                    tau = j * (total_time_sec / total_check_step);
                    graph.add(ObstaclePlanarSDFFactorGPArm( ...
                        key_pos1, key_vel1, key_pos2, key_vel2, ...
                        arm, sdf, cost_sigma, epsilon_dist, ...
                        Qc_model, delta_t, tau));
                end
            end
        end
    end

    % ------------------------- optimize! -------------------------
    use_trustregion_opt = false;

    if use_trustregion_opt
        parameters = DoglegParams;
        parameters.setVerbosity('ERROR');
        optimizer = DoglegOptimizer(graph, init_values, parameters);
    else
        parameters = GaussNewtonParams;
        parameters.setVerbosity('ERROR');
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
    end

    optimizer.optimize();
    result = optimizer.values();
    % result.print('Final results')

    means_gpmp2 = zeros(2, nt+1);
    for i=0:nt
        conf = result.atVector(symbol('x', i));
        means_gpmp2(:, i+1) = conf;
    end
    % csvwrite("zt_gpmp2.csv", means_gpmp2);

end

