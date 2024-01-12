classdef DoubledJoint < HebiArmPlugin
    % DoubledJointMirror
    %
    % Utility plugin to work with joints that have two actuators, e.g.,
    % a double-shoulder configuration. Hrdf currently only implements
    % serial chains, so we can't have a parallel joint in the HebiArm.
    %
    % This plugin lets users treat parallel joints like a normal arm, and
    % it mirrors the commands for the desired joint appropriately. This
    % plugin should be the last in the plugin chain.
    %
    % TODO: the feedback state is currently not changed. We may want to
    % change the effort feedback to be a sum of both actuators.
    
    properties
        group HebiGroup;
        index double;
        sign double;
    end
    
    properties(Access = private)
        cmd = CommandStruct();
    end
    
    methods
        
        function this = DoubledJoint(index, group, mirror)
            this.index = index;
            this.group = group;
            if mirror
                this.sign = -1;
            else
                this.sign = +1;
            end
        end
        
        function [] = update(this, arm)
            
            % Position is negative
            if isempty(arm.state.cmdPos)
               this.cmd.position = [];
            else
               this.cmd.position = this.sign * arm.state.cmdPos(this.index); 
            end
            
            % Velocity is negative
            if isempty(arm.state.cmdVel)
               this.cmd.velocity = [];
            else
               this.cmd.velocity = this.sign * arm.state.cmdVel(this.index); 
            end
            
            % Effort is negative and half-ed
            if isempty(arm.state.cmdEffort)
               this.cmd.effort = [];
            else
               arm.state.cmdEffort(this.index) = arm.state.cmdEffort(this.index) / 2; 
               this.cmd.effort = this.sign * arm.state.cmdEffort(this.index); 
            end
            
            % Send immediately (note that this assume that there are no
            % subsequent changes to the commands after the plugins have
            % run)
            this.group.send(this.cmd);
            
        end
        
    end
    
end

