%IBVS   Implement classical IBVS for point features
%
% A concrete class for simulation of image-based visual servoing (IBVS), a subclass of
% VisualServo.  Two windows are shown and animated:
%   - The camera view, showing the desired view (*) and the 
%     current view (o)
%   - The external view, showing the target points and the camera
%
% Methods::
% run            Run the simulation, complete results kept in the object
% plot_p         Plot image plane coordinates of points vs time
% plot_vel       Plot camera velocity vs time
% plot_camera    Plot camera pose vs time
% plot_jcond     Plot Jacobian condition vs time 
% plot_z         Plot point depth vs time
% plot_error     Plot feature error vs time
% plot_all       Plot all of the above in separate figures
% char           Convert object to a concise string
% display        Display the object as a string
%
% Example::
%         cam = CentralCamera('default');
%         Tc0 = transl(1,1,-3)*trotz(0.6);
%         pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], cam.pp');
%         ibvs = IBVS(cam, 'T0', Tc0, 'pstar', pStar)
%         ibvs.run();
%         ibvs.plot_p();
%
% References::
% - Robotics, Vision & Control, Chap 15
%   P. Corke, Springer 2011.
%
% Notes::
% - The history property is a vector of structures each of which is a snapshot at
%   each simulation step of information about the image plane, camera pose, error, 
%   Jacobian condition number, error norm, image plane size and desired feature 
%   locations.
%
% See also VisualServo, PBVS, IBVS_l, IBVS_e.

% IMPLEMENTATION NOTE
%
% 1.  As per task function notation (Chaumette papers) the error is
%     defined as actual-demand, the reverse of normal control system
%     notation.
% 2.  The gain, lambda, is always positive
% 3.  The negative sign is written into the control law

classdef IBVS < VisualServo

    properties
        lambda          % IBVS gain
        lambda_z
        lambda_wz
        eterm
        eterm1
        uv_p            % previous image coordinates
        Td
        depth
        depthest
        vel_p
        theta
        smoothing
    end

    methods

        function ibvs = IBVS(cam, varargin)
            %IBVS.IBVS Create IBVS visual servo object
            %
            % IB = IBVS(camera, options)
            %
            % Options::
            % 'niter',N         Maximum number of iterations
            % 'eterm',E         Terminate when norm of feature error < E
            % 'lambda',L        Control gain, positive definite scalar or matrix
            % 'T0',T            The initial pose
            % 'P',p             The set of world points (3xN)
            % 'targetsize',S    The target points are the corners of an SxS square
            % 'pstar',p         The desired image plane coordinates
            % 'depth',D         Assumed depth of points is D (default true depth
            %                   from simulation is assumed)
            % 'depthest'        Run a simple depth estimator
            % 'fps',F           Number of simulation frames per second (default t)
            % 'verbose'         Print out extra information during simulation
            %
            % Notes::
            % - If 'P' is specified it overrides the default square target.
            %
            % See also VisualServo.

            % invoke superclass constructor
            ibvs = ibvs@VisualServo(cam, varargin{:});

            % handle arguments
            opt.eterm = 0.5;
            opt.eterm1=0.1;
%             opt.lambda = 0.5;         % control gain 0.1
%             opt.lambda_z=0.5;         % 0.2
             opt.lambda = 0.5;         % control gain 0.1
            opt.lambda_z=0.5;         % 0.2
            opt.lambda_wz=1;
            opt.depth = [];
            opt.depthest = false;
            opt.example = false;
            
            
            opt = tb_optparse(opt, ibvs.arglist);
% Initial point co-ordinates have been defined in VisualServo.m, line 102            
            if opt.example
%                 % run a canned example
                fprintf('---------------------------------------------------\n');
                fprintf('canned example, image-based IBVS with 4 points\n');
                fprintf('---------------------------------------------------\n');
                ibvs.P = mkgrid(2, 0.5, 'pose', SE3(0,0,3));
                ibvs.pf = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], cam.pp');
                ibvs.T0 = SE3(1,1,-3)*SE3.Rz(0.6);
                ibvs.lambda = opt.lambda;
                ibvs.lambda_z = opt.lambda_z;
                ibvs.lambda_wz = opt.lambda_wz;
                ibvs.eterm = 0.5;
                ibvs.eterm1=0.1;
            
            else
                % copy options to IBVS object
                ibvs.lambda = opt.lambda;
                ibvs.lambda_z=opt.lambda_z;
                ibvs.lambda_wz = opt.lambda_wz;
                ibvs.eterm = opt.eterm;
                ibvs.eterm1=opt.eterm1;
                ibvs.theta = 0;
                ibvs.smoothing = 0.80;
                ibvs.depth = opt.depth;
                ibvs.depthest = opt.depthest;
            end
            
            clf
            subplot(121);
            ibvs.camera.plot_create(gca)
            
            % this is the 'external' view of the points and the camera
            subplot(122)
           
            % Creating the rgb landmarks
            hold on
            ibvs.P(:,1);
            ibvs.P(:,2);
            ibvs.P(:,3);
            
            plot_sphere(ibvs.P(:,1), 0.01,'r');
            plot_sphere(ibvs.P(:,2),0.01,'g');
            plot_sphere(ibvs.P(:,3),0.01,'b');
            ibvs.camera.plot_camera(ibvs.P(:,1), 'label');
            ibvs.camera.plot_camera(ibvs.P(:,2), 'label');
            ibvs.camera.plot_camera(ibvs.P(:,3), 'label');
            legend('LED_r','LED_g','LED_b');
            plotvol([-0.2 0.2 -0.2 0.2 -0.2 0.8])
            xticks([-0.2 0 0.2]);
            yticks([-0.2 0 0.2]);
            zticks([-0.2 0 0.2 0.4 0.6 0.8]);
            view(16, 28);
            grid minor
            set(gcf, 'Color', 'w')
            set(gcf, 'HandleVisibility', 'Off');
            ibvs.type = 'point';

        end

        function init(vs)
            %IBVS.init Initialize simulation
            %
            % IB.init() initializes the simulation.  Implicitly called by
            % IB.run().
            %
            % See also VisualServo, IBVS.run.

            if ~isempty(vs.pf)
                % final pose is specified in terms of image coords
                vs.uv_star = vs.pf
                % Saves the co=ordinates of the defined cam pose in
                % IBVS_3pt_landmark
             
            else
                if ~isempty(vs.Tf)
                    vs.Tf = transl(0, 0, 1);
                    warning('setting Tf to default');
                 
                end
                % final pose is specified in terms of a camera-target pose
                %   convert to image coords
                vs.uv_star = vs.camera.project(vs.P, 'Tcam', inv(vs.Tf));
            end
            
            % initialize the vservo variables
            vs.camera.T = vs.T0;    % set camera back to its initial pose
            vs.Tcam = vs.T0;               % initial camera/robot pose
            vs.Tcam;
            
            % show the reference location, this is the view we wish to achieve
            % when Tc = Tct_star
            vs.vel_p = [];
            vs.uv_p = [];
            vs.history = [];
            vs.history1=[];
            vs.history2=[];
        end

        function status_x = step1(vs)
            %IBVS.step Simulate one time step
            %
            % STAT = IB.step() performs one simulation time step of IBVS.  It is
            % called implicitly from the superclass run method.  STAT is
            % one if the termination condition is met, else zero.
            %
            % See also VisualServo, IBVS.run.
            
            status_x = 0;
            Zest = [];
      
            init=vs.P1;
                     
            % compute the view
            P_view = vs.camera.plot(vs.P);
            uv1=P_view(1,:);
            uv2 =init(2,:); %%% else init(2,:)
            uv=[uv1;uv2];
            
            % optionally estimate depth
            if vs.depthest
                % run the depth estimator
                [Zest,Ztrue] = vs.depth_estimator(uv);
                if vs.verbose
                    fprintf('Z: est=%f, true=%f\n', Zest, Ztrue);
                end
                vs.depth = Zest;
                hist.Ztrue = Ztrue(:);
                hist.Zest = Zest(:);
            end
                       
             e1=uv(1,:)-vs.uv_star(1,:);
             e2=init(2,:)-vs.uv_star(2,:);
             e=[e1;e1];
             e=e(:);                   
        
            % compute the Jacobian
            if isempty(vs.depth)
                % exact depth from simulation (not possible in practice)
                pt = inv(vs.Tcam) * vs.P;
                J = vs.camera.visjac_p(uv, pt(3,:) );
%                 J(:,2:6)=0;
            elseif ~isempty(Zest)
                J = vs.camera.visjac_p(uv, Zest);
            else
                J = vs.camera.visjac_p(uv, vs.depth );
            end
           
            % compute the velocity of camera in camera                    
            try
                vs.lambda;
                v = -vs.lambda * pinv(J) * e;
            catch
                status_x = -1;
                return
            end

            if vs.verbose
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            end
            
            % update the camera pose
            Td = SE3(trnorm(delta2tr(v)));    % differential motion

            %Td = expm( skewa(v) );
            %Td = SE3( delta2tr(v) );
            vs.Tcam = vs.Tcam .* Td;       % apply it to current pose
            
            % update the camera pose
            vs.camera.T = vs.Tcam;
            vs.Tcam;

            % update the history variables
            hist.f = uv(:);
            vel = tr2delta(Td);
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e1);
            hist.jcond = cond(J);
            hist.Tcam = vs.Tcam;
            vs.history = [vs.history hist];      
            vs.vel_p = vel;
            vs.uv_p = uv;
      
%             if e(1,1) < vs.eterm
              if e(1,1) < 0.5
                status_x = 1;
                return
            end
        end

   function status_y = step2(vs)
            %IBVS.step Simulate one time step
            %
            % STAT = IB.step() performs one simulation time step of IBVS.  It is
            % called implicitly from the superclass run method.  STAT is
            % one if the termination condition is met, else zero.
            %
            % See also VisualServo, IBVS.run.
            
            status_y = 0;
            Zest = [];
      
            init=vs.P1;
            
            % compute the view
            P_view = vs.camera.plot(vs.P);
            uv1=vs.P(1,:);
            uv2 =P_view(2,:); %%% else init(2,:)
            uv=[uv1;uv2];
            
            % optionally estimate depth
            if vs.depthest
                % run the depth estimator
                [Zest,Ztrue] = vs.depth_estimator(uv);
                if vs.verbose
                    fprintf('Z: est=%f, true=%f\n', Zest, Ztrue);
                end
                vs.depth = Zest;
                hist.Ztrue = Ztrue(:);
                hist.Zest = Zest(:);
            end      

             e1=init(1,:)-vs.uv_star(1,:);
             e2=uv(2,:)-vs.uv_star(2,:);
             e=[e2;e2];
             e=e(:);                     
        
            % compute the Jacobian
            if isempty(vs.depth)
                % exact depth from simulation (not possible in practice)
                pt = inv(vs.Tcam) * vs.P;
                J = vs.camera.visjac_p(uv, pt(3,:) );
%                 J(:,1)=0;
%                 J(:,3:6)=0;
            elseif ~isempty(Zest)
                J = vs.camera.visjac_p(uv, Zest);
            else
                J = vs.camera.visjac_p(uv, vs.depth );
            end
           
            % compute the velocity of camera in camera                 
            try
                vs.lambda;
                v = -vs.lambda * pinv(J) * e;
            catch
                status_y = -1;
                return
            end

            if vs.verbose
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            end
            
            % update the camera pose
            Td = SE3(trnorm(delta2tr(v)));    % differential motion

            %Td = expm( skewa(v) );
            %Td = SE3( delta2tr(v) );
            vs.Tcam = vs.Tcam .* Td;       % apply it to current pose
 
            % update the camera pose
            vs.camera.T = vs.Tcam;
            vs.Tcam;

            % update the history variables
            hist.f = uv(:);
            vel = tr2delta(Td);
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e2);
            hist.jcond = cond(J);
            hist.Tcam = vs.Tcam;
            vs.history = [vs.history hist];
            vs.vel_p = vel;
            vs.uv_p = uv;
                 
            if e2(1,1) < vs.eterm
                status_y = 1;
                return
            end          
   end
    
   function status_z = step3(vs)
            %IBVS.step Simulate one time step
            %
            % STAT = IB.step() performs one simulation time step of IBVS.  It is
            % called implicitly from the superclass run method.  STAT is
            % one if the termination condition is met, else zero.
            %
            % See also VisualServo, IBVS.run.
            
            status_z = 0;
            Zest = [];
           init=vs.P1;
            dist0=20; % in cm

            % compute the view
            % here the distance between the end points have been compared
            % with the desired value based on the acceptable distance
            % between the two robots
            
            uv = vs.camera.plot(vs.P);
            uv1=uv(:,1);
            uv2 =uv(:,3); %%% else init(2,:)
            dif=uv2-uv1;
            pixel_dist=sqrt((dif(1,1)*dif(1,1))+(dif(2,1)*dif(2,1)));
            dist_pt=pixel_dist*(2.54/141.379)
            dist=dist_pt;
            e=dist0-dist                             
        
            % compute the Jacobian
            if isempty(vs.depth)
                % exact depth from simulation (not possible in practice)
                pt = inv(vs.Tcam) * vs.P;
                J = vs.camera.visjac_p(uv, pt(3,:) ); %pt(3,:)
%                 J(:,1:2)=0;
%                 J(:,4:6)=0;
            elseif ~isempty(Zest)
                J = vs.camera.visjac_p(uv, Zest);
            else
                J = vs.camera.visjac_p(uv, vs.depth );
            end
         
            % compute the velocity of camera in camera           
            try
                vs.lambda;
                v = -vs.lambda_z * pinv(J) * e;  %%%%%%%%%%%%%%%% -ve has been removed
            catch
                status_z = -1;
                return
            end

            if vs.verbose
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            end
            
            % update the camera pose
            Td = SE3(trnorm(delta2tr(v)));    % differential motion

            %Td = expm( skewa(v) );
            %Td = SE3( delta2tr(v) );
            vs.Tcam = vs.Tcam .* Td;       % apply it to current pose

            % update the camera pose
            vs.camera.T = vs.Tcam;
            vs.Tcam;
            
             e1=uv(1,:)-vs.uv_star(1,:);
             e2=init(2,:)-vs.uv_star(2,:);
             e=[e1;e1];
             e=e(:);
             
            % update the history variables
            hist.f = uv(:);
            vel = tr2delta(Td);
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e);
            hist.jcond = cond(J);
            hist.Tcam = vs.Tcam;
            vs.history = [vs.history hist];
            vs.vel_p = vel;
            vs.uv_p = uv;
           
%             if norm(pt(3,1)) < vs.eterm1
              if e(1,1) < vs.eterm
                status_z = 1;
                return
            end            
   end 
   
      function status_wz = step4(vs)
            %IBVS.step Simulate one time step
            %
            % STAT = IB.step() performs one simulation time step of IBVS.  It is
            % called implicitly from the superclass run method.  STAT is
            % one if the termination condition is met, else zero.
            %
            % See also VisualServo, IBVS.run.
            
            status_wz = 0;
            Zest = [];
          
            uv = vs.camera.plot(vs.P);
            p12=uv(1,2)-uv(1,1);
            p23=uv(1,3)-uv(1,2);

            % compute the view
            e=p12-p23;
            
            % optionally estimate depth
            if vs.depthest
                % run the depth estimator
                [Zest,Ztrue] = vs.depth_estimator(uv);
                if vs.verbose
                    fprintf('Z: est=%f, true=%f\n', Zest, Ztrue);
                end
                vs.depth = Zest;
                hist.Ztrue = Ztrue(:);
                hist.Zest = Zest(:);
            end                                   
        
            % compute the Jacobian
            if isempty(vs.depth)
                % exact depth from simulation (not possible in practice)
                pt = inv(vs.Tcam) * vs.P;
                J = vs.camera.visjac_p(uv, pt(3,:) ); %pt(3,:)
%                 J(:,1:4)=0;
%                 J(:,6)=0;
            elseif ~isempty(Zest)
                J = vs.camera.visjac_p(uv, Zest);
            else
                J = vs.camera.visjac_p(uv, vs.depth );
            end
         
            % compute the velocity of camera in camera           
            try
                v = -vs.lambda_wz * pinv(J) * e;  %%%%%%%%%%%%%%%% -ve has been removed
            catch
                status_wz = -1;
                return
            end

            if vs.verbose
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
            end
            
            % update the camera pose
            Td = SE3(trnorm(delta2tr(v)));    % differential motion

            %Td = expm( skewa(v) );
            %Td = SE3( delta2tr(v) );
            vs.Tcam = vs.Tcam .* Td;       % apply it to current pose
            
            % update the camera pose
            vs.camera.T = vs.Tcam; 
            vs.Tcam;

            % update the history variables
            hist.f = uv(:);
            vel = tr2delta(Td);
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e);
            hist.jcond = cond(J);
            hist.Tcam = vs.Tcam;
            vs.history = [vs.history hist];
            vs.vel_p = vel;
            vs.uv_p = uv;
           
            if abs(e) < 0.1
                status_wz = 1;
                return
            end             
    end 
   
    function [Zest,Ztrue] = depth_estimator(vs, uv)
            %IBVS.depth_estimator Estimate point depth
            %
            % [ZE,ZT] = IB.depth_estimator(UV) are the estimated and true world 
            % point depth based on current feature coordinates UV (2xN).
            
            if isempty(vs.uv_p)
                Zest = [];
                Ztrue = [];
                return;
            end

            % compute Jacobian for unit depth, z=1
            J = vs.camera.visjac_p(uv, 1);
            Jv = J(:,1:3);  % velocity part, depends on 1/z
            Jw = J(:,4:6);  % rotational part, indepedent of 1/z

            % estimate image plane velocity
            uv_d =  uv(:) - vs.uv_p(:);
            
            % estimate coefficients for A (1/z) = B
            B = uv_d - Jw*vs.vel_p(4:6);
            A = Jv * vs.vel_p(1:3);

            AA = zeros(numcols(uv), numcols(uv)/2);
            for i=1:numcols(uv)
                AA(i*2-1:i*2,i) = A(i*2-1:i*2);
            end
            eta = AA\B;          % least squares solution

            eta2 = A(1:2) \ B(1:2);

            % first order smoothing
            vs.theta = (1-vs.smoothing) * 1./eta' + vs.smoothing * vs.theta;
            Zest = vs.theta;

            % true depth
            P_CT = inv(vs.Tcam) * vs.P;
            Ztrue = P_CT(3,:);

            if vs.verbose
                fprintf('depth %.4g, est depth %.4g, rls depth %.4g\n', ...
                    Ztrue, 1/eta, Zest);
            end
        end
    end % methods
end % class
