function [ pinv_J_damp ] = pinv_damped( J )
 
    if size(J,1) > size(J,2)
        w = det(J'*J);
        I = eye(size(J'*J));
    else
        w = det(J*J');
        I = eye(size(J*J'));
    end
        
    
    % Damping Terms
    % Based on Manipulability, in 'Prior Work' of above paper
    a0 = .01 / 2;    % Max damping factor (no damping = 0)
    w0 = .001 / 2;     % Manipulability threshold for when to start damping
    
    if w < w0
        % If manipulability is less than some threshold, ramp up damping
    	damping = a0 * (1 - w/w0)^2;
    else
        % Otherwise, no damping.
        damping = 0;
    end
    
    
    if size(J,1) > size(J,2)
        pinv_J_damp = (J'*J +  + damping*I) \ J';
    else
        pinv_J_damp = J' / (J*J' + damping*I);
    end
 
end