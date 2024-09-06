kin = robot_kin.kuka;
SEW = sew_conv([0;0;1]);

kin.P(:,end) = 0;
%%

[R, T] = fwdkin(kin, rand_angle([7 1]));

N = 100;
lambda = linspace(-pi, pi, N);

Q = NaN([7 8 N]);
for i = 1:N
    Q(:,:,i) = SEW_IK.IK_2R_2R_3R(R, T, SEW, lambda(i), kin);
end

plot(squeeze(Q(6,:,:))');

%%
syms q [7 1] real

J = robotjacobian(kin, q);
Jp = J(4:6, :);

D = simplify(det(Jp*Jp'));

simplify(D, 'IgnoreAnalyticConstraints',true)
%%
D1 = simplify(det(J(:,1:end ~= 1)))
D2 = simplify(det(J(:,1:end ~= 2)))
D3 = simplify(det(J(:,1:end ~= 3)))
D4 = simplify(det(J(:,1:end ~= 4)))
D5 = simplify(det(J(:,1:end ~= 5)))
D6 = simplify(det(J(:,1:end ~= 6)))
D7 = simplify(det(J(:,1:end ~= 7)))

function J = robotjacobian(kin, theta)
    
    
    p = kin.P(:,1);
    R = eye(3);
    
    J = zeros(6,numel(kin.joint_type), 'like', theta);

    hi = zeros(3,numel(kin.joint_type), 'like', theta);
    pOi = zeros(3,numel(kin.joint_type)+1, 'like', theta);
    pOi(:,1) = p;

    % Compute and store forward kinematics
    for i = 1:numel(kin.joint_type)
        if (kin.joint_type(i) == 0 || ...       % rotational actuators
                    kin.joint_type(i) == 2)        
            R = R*rot(kin.H(:,i),theta(i));
        elseif (kin.joint_type(i) == 1 || ...   % translational actuators
                    kin.joint_type(i) == 3) 
            p = p + R*kin.H(:,i)*theta(i);
        end
        p = p + R*kin.P(:,i+1);
        pOi(:,i+1) = p;
        hi(:,i) = R*kin.H(:,i);
    end

    pOT = pOi(:,end);
    % Compute Jacobian
    i = 1;
    j = 1;
    while i <= numel(kin.joint_type)
        if kin.joint_type(i) == 0               % revolute actuators
            J(:,j) = [hi(:,i); hat(hi(:,i))*(pOT - pOi(:,i))];
        elseif kin.joint_type(i) == 1           % prismatic actuators  
            J(:,j) = [0;0;0; hi(:,i)];
        elseif kin.joint_type(i) == 3           % nonholonomic mobile
            % This is a special case, and is dependent on the next
            % two 'joints' following the format for the unicycle model.  
            % Should consider new format in future release.
            % Linear Velocity
            J(:,j) = [0;0;0;rot(hi(:,i+2),theta(i+2))*hi(:,i)];
            % Angular Velocity
            J(:,j+1) = [hi(:,i+2);hat(hi(:,i+2))*(pOT - pOi(:,i+2))];
            J = J(:,1:end-1);
            i = i + 2;
            j = j + 1;
        end
        i = i + 1;
        j = j + 1;
    end
end