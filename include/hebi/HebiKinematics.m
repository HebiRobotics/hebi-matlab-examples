classdef (Sealed) HebiKinematics
    % HebiKinematics provides basic kinematic methods for HEBI modules
    %
    %   HebiKinematics loads HRDF files that describe the configuration of
    %   a robot and helps with calculating things like forward kinematics,
    %   inverse kinematics, Jacobians, as well as forces and torques to
    %   compensate for accelerations due to gravity or dynamic motions.
    %
    %   More information and background on kinematics:
    %   http://docs.hebi.us/core_concepts.html#kinematics
    %
    %   More information on the HEBI Robot Description Format (HRDF):
    %   http://docs.hebi.us/tools.html#robot-description-format
    %
    %   This API currently only supports serial chains. If you are going to
    %   work with a robot that has multiple limbs, such as a hexapod, we
    %   recommend creating a cell array that contains a separate kinematic
    %   object for each limb. The base frames can be set to the pose of the
    %   first body of the limb with respect to the chassis.
    %
    %   HebiKinematics Methods (setup):
    %      kin = HebiKinematics('robot.hrdf') - where 'robot.hrdf' is the
    %                                           path to the file that
    %                                           describes the robot.
    %
    %   HebiKinematics Methods (kinematics):
    %      getForwardKinematics  - calculates the pose of bodies in the
    %                              kinematic chain, given a set of joint
    %                              positions
    %      getInverseKinematics  - calculates the required joint positions to
    %                              generate a desired end-effector pose
    %      getJacobian           - calculates the matrix that relates joint
    %                              velocities to body velocities
    %      getGravCompEfforts    - calculates the efforts that compensate
    %                              for gravitational accelerations, given
    %                              joint positions and gravity vector.
    %      getDynamicCompEfforts - calculates the efforts that compensate
    %                              for dynamics of a desired motion, given
    %                              joint position/velocities/accelerations
    %      setBaseFrame          - set transform from world to first body
    %                              in the kinematic chain
    %      setPayload            - sets an additional mass at the end-effector
    %                              for effort compensation
    %
    %   HebiKinematics Methods (information):
    %      getNumBodies          - number of bodies
    %      getNumDoF             - number of degrees of freedom
    %      getBodyMasses         - a vector of all body masses [kg]
    %      getBodyInfo           - a table of body related info
    %      getJointInfo          - a table of joint related info
    %      getBaseFrame          - get transform from world to first body
    %      getPayload            - additional mass at end-effector used 
    %                              for effort compensation
    %
    %   HebiKinematics Methods (programmatic setup):
    %      addBody               - adds a body to the end of the chain.
    %                              THIS IS NO LONGER THE PREFERRED METHOD
    %                              OF DEFINING A ROBOT CONFIGURATION. It
    %                              is better to make and load an HRDF file.
    %
    %   Example
    %      % Load model from file (experimental support for hrdf v1.1)
    %      kin = HebiKinematics('robot.hrdf');
    %
    %   Example
    %      % Calculate forward kinematics for some random joint positions
    %      positions = rand(kin.getNumDoF, 1);
    %      frames = kin.getForwardKinematics('output', positions);
    %
    %   See also HebiGroup
    
    %   Copyright 2014-2018 HEBI Robotics, Inc.
    
    % Public API
    methods(Access = public)
        
        function this = addBody(this, varargin)
            % addBody adds a body to the end of the chain
            %
            %   THIS METHOD IS NO LONGER THE PREFERRED WAY OF SETTING UP A
            %   NEW KINEMATICS OBJECT. PLEASE LOAD FROM AN .HRDF FILE.  FOR
            %   MORE INFORMATION, PLEASE SEE:
            %   http://docs.hebi.us/tools.html#robot-description-format
            %
            %   This method creates a serial chain of bodies that describe
            %   the kinematic relation of a robot. A 'body' can be a rigid
            %   link as well as a dynamic element. More detailed
            %   documentation on body types and parameters can be found at:
            %   http://docs.hebi.us/hardware.html#Kinematic_Info
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
            %       'X8-16'
            %       'X5-Link'               (Extension, Twist)
            %       'X5-LightBracket'       (Mounting)
            %       'X5-HeavyBracket'       (Mounting)
            %
            %     Custom Types
            %       'GenericJoint'          (Axis)
            %       'GenericLink'           (CoM, OutputTransform, Mass )
            %
            %   Some types may require a set of parameters. Parameters
            %   that are not required by the specified type are ignored.
            %   Potential kinematic parameters include:
            %
            %       Parameter          Size    Units      Synonyms
            %       'Extension'        1x1     [m]        ('ext')
            %       'Twist'            1x1     [rad]
            %       'Mass'             1x1     [kg]
            %       'CoM'              3x1     [m]
            %       'Axis'             1x1     [tx|ty|tz|rx|ry|rz]
            %       'OutputTransform'  4x4                ('output','out')
            %       'Mounting'         1x1     [left|...] ('mount')
            %
            %   Additionally, there are optional parameters that can be used
            %   to control the behavior of joints when calculating inverse
            %   kinematics with HEBIKINEMATICS, or generating trajectories
            %   with HEBITRAJECTORYGENERATOR. Joint limits are applicable
            %   to all joints and expect a [min max] vector without NaN.
            %
            %       Parameter          Size    Units      Synonyms
            %       'PositionLimit'    1x2     [rad|m]    ('PosLim')
            %       'VelocityLimit'    1x2     [rad/s]    ('VelLim')
            %       'EffortLimit'      1x2     [Nm|N]     ('EffLim')
            %       'Mass'             1x1     [kg]
            %
            %   Example
            %      % Setup a common 5 DoF X-Series arm configuration
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
            %   More general info on kinematics can be found at:
            %   http://docs.hebi.us/core_concepts.html#kinematics
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
        
        function mass = getPayload(this, varargin)
            % getPayload returns the payload used for effort compensation
            %
            %   This method returns the specified payload mass at the
            %   end-effector that gets used to calculate efforts to
            %   compensate for gravitational effects and joint
            %   accelerations.
            %   
            %   Example
            %       mass = 0.1; % [kg]
            %       kin.setPayload(mass, 'CoM', com);
            %       mass = kin.getPayload(); % [kg]
            %
            %   See also HebiKinematics, setPayload,
            %   getGravCompEfforts, getDynamicCompEfforts, 
            mass = getPayloadMass(this.obj, varargin{:});
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
            % getForwardKinematics (getFK) calculates the poses of all the
            % bodies in the configured kinematic chain, setup by using
            % HebiKinematics() and an HRDF file.
            %
            %   This method computes the poses of the chain of bodies in
            %   the world frame, using specified values for the joint
            %   parameters and specified base frame.
            %
            %   Poses are returned as a set of [4 x 4 x numBodies]
            %   homogeneous transforms, specified in the world frame of the
            %   kinematic configuration.  Units of XYZ translation are in
            %   [m].
            %
            %   'FrameType' Argument
            %      'OutputFrame'      calculates the transforms to the output
            %                         of each body ('out')
            %                         Size: [4 x 4 x numBodies]
            %
            %      'CoMFrame'         calculates the transforms to the center
            %                         of mass of each body
            %                         Size: [4 x 4 x numBodies]
            %
            %      'EndEffectorFrame' calculates the transform to only the
            %                         output frame of the last body in the
            %                         chain (e.g. a gripper or tool tip)
            %                         Size: [4 x 4]
            %
            %   'Position' Argument
            %      A [1 x numDoF] vector that specifies the position of
            %      each degree of freedom.  Rotational positions are
            %      specified in [rad].  Linear positions are in [m].
            %
            %   Examples:
            %      % Forward kinematics for all the bodies in an arm
            %      % using group feedback.
            %      fbk = group.getNextFeedback();
            %      frames = kin.getFK('output', fbk.position);
            %
            %      % Forward kinematics for just the endeffector of an arm
            %      % using commanded positions.
            %      fbk = group.getNextFeedback();
            %      frames = kin.getFK('endeffector', fbk.positionCmd);
            %
            %   See also HebiKinematics, getFK, addBody, setBaseFrame
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
            % for getForwardKinematics('EndEffectorFrame').
            %
            %   See also HebiKinematics, getForwardKinematics
            out = getForwardKinematicsEndEffector(this.obj, varargin{:});
        end
        
        function out = getInverseKinematics(this, varargin)
            % getInverseKinematics (getIK) calculates positions for a
            % desired end effector pose.
            %
            %   This method computes the joint positions associated to a
            %   desired end-effector configuration. The end effector is
            %   assumed to be the last body in the kinematic chain.
            %
            %   getInverseKinematics uses a gradient-descent based local
            %   optimizer to find a valid configuration.
            %
            %   'InitialPositions' ('Initial') provides the seed for the
            %   numerical optimization. IT IS HIGHLY RECOMMENDED THAT YOU
            %   SPECIFY SEED POSITIONS, AND A FUTURE VERSION OF THE API
            %   WILL MAKE THIS A REQUIRED PARAMETER.
            %
            %   There are a variety of optimization criteria that can be
            %   combined depending on the application. Available parameters
            %   include:
            %
            %      Parameter       EndEffector Target      Size / Units
            %
            %      'XYZ'           xyz position in [m]     3x1 in [m]
            %
            %      'TipAxis'       z-axis orientation      3x1 unit vector
            %                      of the last body in
            %                      in chain [unit vector]
            %
            %      'SO3'           3-DoF orientation       3x3 rotation
            %                                                  matrix
            %
            %      Note that 'XYZ' supports NaN for dimensions that
            %      should be ignored. For example, a planar arm may use
            %      the target position of [x y NaN].
            %
            %      All target positions and orientations are expressed in
            %      the base frame.
            %
            %   'MaxIterations' ('MaxIter') sets the maximum allowed
            %   iterations of the numerical optimization before returning.
            %   This can prevent IK from taking a long time to run, at the
            %   expense of solutions that are potentially less accurate.
            %   The default value is 150 iterations.
            %
            %   Examples:
            %      % Inverse kinematics for a 3-DoF arm, specifying initial
            %      % joint angle positions.
            %      xyz = [0.2 0.1 0.0];
            %      initialJointAngs = [0 -pi/4 pi/2];
            %      waypoints = kin.getInverseKinematics( 'XYZ', xyz, ...
            %                    'IntialPositions', initialJointAngs );
            %
            %      % Inverse kinematics for a 5-DoF arm, specifying initial
            %      % joint angle positions.
            %      xyz = [0.2 0.1 0.0];
            %      tipAxis = [0 0 -1];  % end effector points straight down
            %      initialJointAngs = [0 -pi/4 pi/2 pi/4 0];
            %      waypoints = kin.getIK( 'XYZ', xyz, ...
            %                             'TipAxis', tipAxis, ...
            %                             'initial', initialJointAngs );
            %
            %      % Inverse kinematics for full 6-DoF arm, using the
            %      % latest feedback as the seed position for IK.
            %      xyz = [0.3 -0.1 0.2];
            %      rotMatrix = eye(3);
            %      fbk = group.getNextFeedback();
            %      positions = kin.getIK( 'XYZ', xyz, ...
            %                             'SO3', rotMatrix, ...
            %                             'initial', fbk.position );
            %
            %   See also HebiKinematics, getIK
            out = getInverseKinematics(this.obj, varargin{:});
        end
        
        function out = getIK(this, varargin)
            % getIK is an abbreviation for getInverseKinematics
            %
            %   See also HebiKinematics, getInverseKinematics
            out = getIK(this.obj, varargin{:});
        end
        
        function out = getJacobian(this, varargin)
            % getJacobian calculates the matrix that relates input joint
            % velocities to body velocities.
            %
            %   This method calculates the partial derivatives of the
            %   kinematics equation, which relates the joint velocities to
            %   the linear and angular velocities of each body in the
            %   kinematic chain.
            %
            %   More background on Jacobians and kinematics can be found at:
            %   http://docs.hebi.us/core_concepts.html#kinematics
            %
            %   The Jacobian is returned as a [6 x numDoF x numBodies] set
            %   of matrices.  Rows 1:3 of the Jacobian correspond to linear
            %   velocities [m/s] along the X-Y-Z axes in the world frame,
            %   while rows 4:6 correspond to rotational velocities [rad/s]
            %   about the X-Y-Z axes in the world frame.
            %
            %   'FrameType' Argument
            %       'OutputFrame'      calculates the transforms to the output
            %                          of each body ('out')
            %
            %       'CoMFrame'         calculates the transforms to the center
            %                          of mass of each body
            %
            %       'EndEffectorFrame' calculates the transform to only the
            %                          output frame of the last body, e.g.,
            %                          a gripper
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
            % for getJacobian('EndEffectorFrame', position).
            %
            %   See also HebiKinematics, getJacobian
            out = getJacobianEndEffector(this.obj, varargin{:});
        end
        
        function out = getGravCompEfforts(this, varargin)
            % getGravCompEfforts calculates joint efforts that compensate
            % for gravity.
            %
            %   This method computes the efforts that are required to
            %   cancel out the forces on an arm caused by gravity
            %
            %   'Positions' argument expects a [1 x numDoF] vector of
            %   positions of all degrees of freedom.
            %
            %   'GravityVector' argument expects an [3 x 1] vector of the
            %   direction of gravity in the base frame.  Note that this
            %   direction vector is not required to be unit length, and the
            %   magnitude of gravitational acceleration is always assumed
            %   to be 9.81 m/s^2.
            %
            %   Example
            %      % Compensate gravity at current feedback positions
            %      fbk = group.getNextFeedback();
            %      gravity = [0 0 -1];
            %      efforts = kin.getGravCompEfforts(fbk.position, gravity);
            %
            %   See also HebiKinematics, setPayload
            out = getGravCompEfforts(this.obj, varargin{:});
        end
        
        function out = getDynamicCompEfforts(this, varargin)
            % getDynamicCompEfforts calculates joint efforts that
            % compensate for dynamic motions
            %
            %   This method computes the efforts that are required to
            %   accelerate the body masses as determined from the specified
            %   positions, velocities, and accelerations.
            %
            %   A recommended way of determining a set of desired
            %   positions, velocities, and accelerations is to use the
            %   HebiTrajectoryGenerator to create minimum-jerk trajectories
            %   for the motion of the system.
            %
            %   'Positions' argument expects a vector of positions of
            %   all degrees of freedom, used for computing the Jacobian,
            %   where (effort = J' * desiredForces)
            %
            %   'TargetPositions', 'TargetVelocities', and
            %   'TargetAccelerations' typically come from some sort of
            %   trajectory generation function, like HebiTrajectoryGenerator
            %   (link in "see also"), or a sine trajectory (example below).
            %
            %   Example
            %      % Compensate for dynamics of sinusoidal motion
            %      fbk = group.getNextFeedback();
            %
            %      time = 0.0;
            %      freq = 1.0 * (2*pi);  % 1 Hz
            %      amp = 1.0; % rad
            %
            %      position = amp * sin( freq * time );
            %      velocity = freq * amp * cos( freq*time );
            %      accel = -freq^2 * amp * sin( freq*time );
            %
            %      cmdPositions = position * ones(1,group.getNumModules);
            %      cmdVelocities = velocity * ones(1,group.getNumModules);
            %      cmdAccelerations = accel * ones(1,group.getNumModules);
            %
            %      efforts = kin.getDynamicCompEfforts(...
            %                                    fbk.position, ...
            %                                    cmdPositions, ...
            %                                    cmdVelocities, ...
            %                                    cmdAccelerations);
            %
            %   See also HebiKinematics, setPayload, HebiTrajectory,
            %   HebiTrajectoryGenerator.
            out = getDynamicCompEfforts(this.obj, varargin{:});
        end
        
    end
    
    methods(Access = public, Hidden = true)
        
        function this = HebiKinematics(varargin)
            % constructor
            this.obj = javaObject(HebiKinematics.className);
            
            if nargin > 0
                addHrdf(this.obj, varargin{1});
            end
            
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
