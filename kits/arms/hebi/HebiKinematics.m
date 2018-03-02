classdef (Sealed) HebiKinematics
    % HebiKinematics provides basic kinematic methods for HEBI modules
    %
    %   This Kinematics API helps with calculating forwards kinematics and
    %   inverse kinematics, Jacobians, as well as forces and torques to
    %   compensate for accelerations due to gravity or dynamic motions.
    %
    %   Note that his API currently only supports serial chains. If you
    %   work with a robot that has multiple limbs, such as a hexapod, we
    %   recommend creating a cell array that contains a separate kinematic
    %   object for each limb. The base frames can be set to the location of
    %   the limb with respect to the chassis.
    %
    %   HebiKinematics Methods (setup):
    %      addBody               -  adds a body to the end of the chain
    %      getNumBodies          -  number of bodies
    %      getNumDoF             -  number of degrees of freedom
    %      getBodyMasses         -  a vector of all body masses [kg]
    %      getBodyInfo           -  a table of body related info
    %      getJointInfo          -  a table of joint related info
    %      setBaseFrame          -  relationship from world to first body
    %
    %   HebiKinematics Methods (kinematics):
    %      getForwardKinematics  -  calculates the configuration of bodies
    %      getJacobian           -  relates joint to body velocities
    %      getInverseKinematics  -  positions for a desired configuration
    %      getGravCompEfforts    -  compensates for gravitational accelerations
    %      getDynamicCompEfforts -  compensates for accelerations due to motions
    %      setPayload            -  sets a payload for effort compensation
    %
    %   Example
    %      % Setup a simple 3 dof arm made of X5 modules
    %      kin = HebiKinematics();
    %      kin.addBody('X5-9');
    %      kin.addBody('X5-LightBracket', 'mounting', 'right');
    %      kin.addBody('X5-4');
    %      kin.addBody('X5-Link', 'extension', 0.350, 'twist', pi/2);
    %      kin.addBody('X5-1');
    %
    %   Example
    %      % Calculate forward kinematics with random inputs
    %      positions = rand(kin.getNumDoF, 1);
    %      frames = kin.getForwardKinematics('output', positions);
    %
    %   See also HebiGroup
    
    %   Copyright 2014-2016 HEBI Robotics, LLC.
    
    % Public API
    methods(Access = public)
        
        function this = addBody(this, varargin)
            % addBody adds a body to the end of the chain
            %
            %   This method creates a serial chain of bodies that describe
            %   the kinematic relation of a robot. A 'body' can be a rigid
            %   link as well as a dynamic element.
            %
            %   The 'Type' argument specifies the type of module or body
            %   that should be added. Currently implemented types include:
            %
            %     X-Series Types            Required Parameters
            %       'X5-1'
            %       'X5-4'
            %       'X5-9'
            %       'X8-3'
            %       'X8-9'
            %       'X5-Link'               (ext,twist)
            %       'X5-LightBracket'       (mount)
            %       'X5-HeavyBracket'       (mount)
            %
            %     S-Series Types
            %       'S5-3'
            %       'S5-Gripper'
            %       'S5-Link'               (ext,twist)
            %       'S5-ElbowLink'          (ext1,twist1,ext2,twist2)
            %
            %     Custom Types
            %       'GenericJoint'          (axis,in,out,com,mass)
            %       'GenericLink'           (com,out,mass)
            %
            %   Some types may require a set of parameters. Parameters
            %   that are not required by the specified type are ignored.
            %   Potential parameters include:
            %
            %       Parameter          Size    Units      Synonyms
            %       'Extension'        1x1     [m]        ('ext','ext1')
            %       'Extension2'       1x1     [m]        ('ext2')
            %       'Twist'            1x1     [rad]      ('twist1')
            %       'Twist2'           1x1     [rad]
            %       'Mass'             1x1     [kg]
            %       'CoM'              3x1     [m]
            %       'Input'            4x4                ('in')
            %       'Output'           4x4                ('out')
            %       'Axis'             1x1     [x|rx|...]
            %       'Mounting'         1x1     [left|...] ('mount')
            %
            %   Additionally, there are optional parameters that can be used
            %   to overwrite default behavior. Joint limits are applicable
            %   to all joints and expect a [min max] vector without NaN.
            %
            %       Parameter          Size    Units      Synonyms
            %       'PositionLimit'    1x2     [rad|m]    ('PosLim')
            %       'VelocityLimit'    1x2     [rad/s]    ('VelLim')
            %       'EffortLimit'      1x2     [Nm|N]     ('EffLim')
            %       'Mass'             1x1     [kg]
            %
            %   Example
            %      % Setup a common 4 dof S-Series arm configuration
            %      inch2m = 0.0254;
            %      kin = HebiKinematics();
            %      kin.addBody('S5-3');
            %      kin.addBody('S5-3');
            %      kin.addBody('S5-ElbowLink', ...
            %          'ext1', 4 * inch2m, 'twist1', pi/2, ...
            %          'ext2', 0.5 * inch2m, 'twist2', pi);
            %      kin.addBody('S5-3');
            %      kin.addBody('S5-Link', ...
            %          'ext', 6 * inch2m, 'twist', -pi/2);
            %      kin.addBody('S5-3');
            %      kin.addBody('S5-Gripper');
            %
            %   Example
            %      % Setup a common 5 dof X-Series arm configuration
            %      kin = HebiKinematics();
            %      kin.addBody('X5-9', 'PosLim', [-pi +pi]);
            %      kin.addBody('X5-HeavyBracket', 'mount', 'right-outside');
            %      kin.addBody('X5-4');
            %      kin.addBody('X5-Link', 'ext', 0.350, 'twist', pi/2);
            %      kin.addBody('X5-4');
            %      kin.addBody('X5-Link', 'ext', 0.250, 'twist', pi/2);
            %      kin.addBody('X5-1');
            %      kin.addBody('X5-LightBracket', 'mount', 'right');
            %      kin.addBody('X5-1');
            %
            %   See also HebiKinematics
            addBody(this.obj, varargin{:});
        end
        
        function this = setPayload(this, varargin)
            % setPayload sets a payload used for effort compensation
            %
            %   This method provides a way to dynamically specify a payload
            %   that gets used to calculate efforts to compensate for
            %   gravitational effects and joint accelerations.
            %
            %   Specifying a payload has no effect on any other
            %   functionality.
            %
            %   The 'Mass' argument (required) specifies the payload mass
            %   in [kg].
            %
            %   The 'CoM' argument (parameter) specifies the distance from
            %   the output to the center of mass of the payload. If left
            %   unspecified, the default assumes that the payload is
            %   located at the output of the end effector.
            %   
            %   Example
            %       mass = 1; % [kg]
            %       com = [1 0 0]; % 1 [m] in x
            %       kin.setPayload(mass, 'CoM', com);
            %
            %   See also HebiKinematics, getGravCompEfforts, 
            %   getDynamicCompEfforts
            setPayload(this.obj, varargin{:});
        end
        
        function out = getNumBodies(this, varargin)
            % getNumBodies returns the total number of bodies
            %
            %   This method returns the total number of bodies of the 
            %   current configuration. This number includes all passive 
            %   and actuated elements.
            %
            %   See also HebiKinematics
            out = getNumBodies(this.obj, varargin{:});
        end
        
        function out = getNumDoF(this, varargin)
            % getNumDoF returns the number of actuated degrees of freedom
            %
            %   This method returns the number of degrees of freedom of the
            %   current kinematics configuration. This is number is also 
            %   the length of the position vector for the kinematics.
            %
            %   See also HebiKinematics
            out = getNumDoF(this.obj, varargin{:});
        end
        
        function out = getBodyMasses(this, varargin)
            % getBodyMasses returns a vector of the masses of all links
            %
            %   This method returns a [numBodies x 1] mass vector that 
            %   contains the weights for each body in [kg].
            %
            %   See also HebiKinematics
            out = getBodyMasses(this.obj, varargin{:});
        end

        function out = getBodyInfo(this, varargin)
            % getBodyInfo returns a table of body info
            %
            %   This method returns a table with numBodies rows that
            %   contains information about the type and mass of each 
            %   body. This information is useful for, e.g., drawing.
            %
            %   Example
            %     % Select info of all non-DoF
            %     info = kin.getBodyInfo();
            %     types = info(~info.isDoF, :);
            %
            %   See also HebiKinematics, getJointInfo
            out = getBodyInfo(this.obj, varargin{:});
            out = struct(out);
            out = rmfield(out, {
                'positionLimit' 
                'velocityLimit' 
                'effortLimit'});
            out = struct2table(out);
            out(end, :) = [];
        end
        
        function out = getJointInfo(this, varargin)
            % getJointInfo returns a table of joint info
            %
            %   This method returns a table with numDoF rows that
            %   contains information about the type, mass, and
            %   limits of each joint. This information is useful
            %   for, e.g., trajectory generation.
            %
            %   Example
            %     % Find position limits of all DoF
            %     info = kin.getJointInfo();
            %     limits = info.positionLimit;
            %
            %   See also HebiKinematics, getBodyInfo
            out = getJointInfo(this.obj, varargin{:});
            out = struct(out);
            out = struct2table(out);
            out(end, :) = [];
            out = out(out.isDoF, :);
            out.isDoF = [];
        end


        function out = getBaseFrame(this, varargin)
            % getBaseFrame returns the relationship between the world
            % and the first body in the kinematic configuration.
            %
            %   This method returns a 4x4 homogeneous transform that
            %   describes the relationship between the world frame and the
            %   frame of the first body. All kinematics are expressed in
            %   the world frame.  Units of XYZ translation are in [m].
            %
            %   See also HebiKinematics, setBaseFrame
            out = getBaseFrame(this.obj, varargin{:});
        end
        
        function this = setBaseFrame(this, varargin)
            % setBaseFrame sets the relationship between the world and the
            % first body in the kinematic configuration.
            %
            %   This method expects a 4x4 homogeneous transform that 
            %   describes the relationship between the world frame and the 
            %   frame of the first body. All kinematics are expressed in 
            %   the world frame.  Units of XYZ translation are in [m].
            %
            %   Example
            %     % Shift the base frame of the kinematics by .5 meters 
            %     % in the +x direction.
            %     newBaseFrame = eye(4);
            %     newBaseFrame(1:3,4) = [.5; 0; 0];
            %     setBaseFrame( newBaseFrame );
            %
            %   See also HebiKinematics, getBaseFrame
            setBaseFrame(this.obj, varargin{:});
        end
        
        function out = getForwardKinematics(this, varargin)
            % getForwardKinematics calculates the pose of all the bodies in
            % the kinematic configuration
            %
            %   This method computes the poses of the chain of bodies in 
            %   the base frame, using specified values for the joint 
            %   parameters.
            %
            %   Poses are returned as a set of [4 x 4 x numBodies] 
            %   homogeneous transforms, specified in the world frame of the
            %   kinematic configuration.  Units of XYZ translation are in 
            %   [m].
            %
            %   'FrameType' Argument
            %      'Output'      calculates the transforms to the output
            %                    of each body ('out')
            %      'CoM'         calculates the transforms to the center of
            %                    mass of each body
            %      'EndEffector' calculates the transform to only the
            %                    output frame of the last body, e.g.,
            %                    a gripper
            %
            %   'Position' Argument
            %      A [1 x numDoF] vector that specifies the position of 
            %      each degree of freedom. Rotational positions are 
            %      specified in [rad].
            %
            %   Example
            %      % Forward kinematics using group feedback
            %      fbk = group.getNextFeedback();
            %      frames = kin.getFK('output', fbk.position);
            %
            %   See also HebiKinematics
            out = getForwardKinematics(this.obj, varargin{:});
        end
        
        function out = getFK(this, varargin)
            % getFK is an abbreviation for getForwardKinematics
            %
            %   See also HebiKinematics, getForwardKinematics
            out = getFK(this.obj, varargin{:});
        end

        function out = getForwardKinematicsEndEffector(this, varargin)
            % getForwardKinematicsEndEffector is a convenience wrapper
            % for getForwardKinematics('EndEffector').
            %
            %   See also HebiKinematics, getForwardKinematics
            out = getForwardKinematicsEndEffector(this.obj, varargin{:});
        end
        
        function out = getInverseKinematics(this, varargin)
            % getInverseKinematics calculates positions for a desired end
            % effector pose.  
            %
            %   This method computes the joint positions associated to a 
            %   desired end-effector configuration. The end effector is 
            %   assumed to be the last body in the kinematic chain. There 
            %   are a variety of optimization criteria that can be combined 
            %   depending on the application. Available parameters include:
            %
            %      Parameter       EndEffector Target     Synonyms
            %      'Xyz'           xyz position
            %      'TipAxis'       z-axis orientation     ('axis')
            %      'SO3'           3-dof orientation
            %
            %   'MaxIterations' ('MaxIter') sets the maximum allowed
            %   iterations of the numerical optimization before returning.
            %
            %   'InitialPositions' ('Initial') provides the seed for the
            %   numerical optimization. By default the  optimization seeds
            %   with zeros.
            %
            %   Note that 'xyz' supports NaN for dimensions that should be
            %   ignored. For example, a planar arm may target [x y nan].
            %
            %   Example
            %      % Inverse kinematics on cartesian coordinates
            %      xyz = [0 0 0];
            %      waypoints = kin.getInverseKinematics('xyz', xyz);
            %
            %      % Inverse kinematics for full 6 dof
            %      xyz = [0 0 0];
            %      so3 = eye(3);
            %      positions = kin.getIK('xyz', xyz, 'so3', so3);
            %
            %   See also HebiKinematics
            out = getInverseKinematics(this.obj, varargin{:});
        end
        
        function out = getIK(this, varargin)
            % getIK is an abbreviation for getInverseKinematics
            %
            %   See also HebiKinematics, getInverseKinematics
            out = getIK(this.obj, varargin{:});
        end
        
        function out = getJacobian(this, varargin)
            % getJacobian calculates the matrix that relates input DoF 
            % velocities to body velocities
            %
            %   This method calculates the partial derivatives of the 
            %   kinematics equation, which relates the joint rates to the 
            %   linear and angular velocity of each body in the kinematic 
            %   configuration.
            %
            %   The Jacobian is returned as a [6 x numDoF x numBodies] set 
            %   of matrices.  Rows 1:3 of the Jacobian correspond to linear 
            %   velocities [m/s] along the X-Y-Z axes in the world frame, 
            %   while rows 4:6 correspond to rotational velocities [rad/s] 
            %   about the X-Y-Z axes in the world frame.
            %
            %   'FrameType' Argument
            %       'Output'      calculates the transforms to the output
            %                     of each body ('out')
            %       'CoM'         calculates the transforms to the center
            %                     of mass of each body
            %       'EndEffector' calculates the transform to only the
            %                     output frame of the last body, e.g.,
            %                     a gripper
            %
            %   'Position' Argument
            %       A [1 x numDoF] vector that specifies the position
            %       of each degree of freedom. Rotational positions are 
            %       specified in [rad].  Translational positions are 
            %       specified in [m].
            %
            %    Example
            %       % End-Effector Jacobian using group feedback
            %       fbk = group.getNextFeedback();
            %       J = kin.getJacobian('endEffector', fbk.position);
            %
            %   See also HebiKinematics
            out = getJacobian(this.obj, varargin{:});
        end

        function out = getJacobianEndEffector(this, varargin)
            % getJacobianEndEffector is a convenience wrapper
            % for getJacobian('EndEffector').
            %
            %   See also HebiKinematics, getJacobian
            out = getJacobianEndEffector(this.obj, varargin{:});
        end
        
        function out = getGravCompEfforts(this, varargin)
            % getGravCompEfforts calculates joint efforts that compensate 
            % for gravity
            %
            %   This method computes the efforts that are required to
            %   cancel out the forces on an arm caused by gravity
            % 
            %   'Positions' argument expects a [1 x numDoF] vector of 
            %   positions of all degrees of freedom.
            % 
            %   'GravityVector' argument expects an [3 x 1] vector of the
            %   direction of gravity in the base frame.  Note that this
            %   direction vector is not required to be unit length, and 
            %   gravitational acceleration is assumed to be 9.81 m/s^2.
            %
            %   Example
            %      % Compensate gravity at current position
            %      fbk = group.getNextFeedback();
            %      gravity = [0 0 -1];
            %      efforts = kin.getGravCompEfforts(fbk.position, gravity);
            %
            %   See also HebiKinematics
            out = getGravCompEfforts(this.obj, varargin{:});
        end
        
        function out = getDynamicCompEfforts(this, varargin)
            % getDynamicCompEfforts calculates joint efforts that 
            % compensate for dynamic motions
            %
            %   This method computes the efforts that are required to
            %   accelerate the body masses as determined from the specified
            %   positions, velocities and accelerations.
            %
            %   'Positions' argument expects a vector of positions of
            %   all degrees of freedom, used for computing the Jacobian,
            %   where (effort = J' * desiredForces)
            %
            %   'TargetPositions', 'TargetVelocities', and 
            %   'TargetAccelerations' typically come from some sort of 
            %   trajectory generation function, such as a minimum-jerk 
            %   trajectory, or a sinusoidal trajectory (example below).
            %
            %   Example
            %      % Compensate for dynamics of sinusoidal motion
            %      fbk = group.getNextFeedback();
            % 
            %      time = 0;
            %      freq = 1 * (2*pi);  % 1 Hz 
            %      amp = 1; 
            %      position = amp * sin( freq * time );
            %      velocity = freq * amp * cos( freq*time );
            %      accel = -freq^2 * amp * sin( freq*time );
            %      cmdPositions = position * ones(1,group.getNumModules);
            %      cmdVelocities = velocity * ones(1,group.getNumModules);
            %      cmdAccelerations = accel * ones(1,group.getNumModules);
            %      efforts = kin.getDynamicCompEfforts(...
            %                                    fbk.position, ...
            %                                    cmdPositions, ...
            %                                    cmdVelocities, ...
            %                                    cmdAccelerations);
            %
            %   See also HebiKinematics
            out = getDynamicCompEfforts(this.obj, varargin{:});
        end
        
    end
    
    methods(Access = public, Hidden = true)
        
        function this = HebiKinematics()
            % constructor
            this.obj = javaObject(HebiKinematics.className);
        end
        
        function disp(this)
            % custom display
            disp(this.obj);
        end
        
    end
    
    properties(Access = public, Hidden = true)
        obj
    end
    
    properties(Constant, Access = private, Hidden = true)
        className = hebi_load('HebiKinematics');
    end
    
    % Non-API Static methods for MATLAB compliance
    methods(Access = public, Static, Hidden = true)
        
        function varargout = methods(varargin)
            instance = javaObject(HebiKinematics.className);
            switch nargout
                case 0
                    methods(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = methods(instance, varargin{:});
            end
        end
        
        function varargout = fields(varargin)
            instance = javaObject(HebiKinematics.className);
            switch nargout
                case 0
                    fields(instance, varargin{:});
                    varargout = {};
                otherwise
                    varargout{:} = fields(instance, varargin{:});
            end
        end
        
    end
    
end
