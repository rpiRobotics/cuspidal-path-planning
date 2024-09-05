syms q [3 1] real

kin = cuspidal_3R.get_kin;

J = robotjacobian(kin, q);
Jp = J(4:6,:);
dJ = det(Jp);
dJ = simplify(dJ)
%%
t2 = solve(dJ, q2)

syms T

T3 = cos(T)
T2 = cos(t2)
T2 = subs(T2, q3, T3)

%%

[~, p] = fwdkin(kin, q)

r = sqrt(p(1)^2+p(2)^2)
r = simplify(r)
z = p(3)
%%
r12 = subs(r, q2, t2)
z12 = subs(z, q2, t2)

fp1 = fplot(r12(1), z12(1), 'k'); hold on
fp2 = fplot(r12(2), z12(2), 'k'); hold off

fp1.MeshDensity = 100;
fp2.MeshDensity = 100;

axis equal

%%%

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
   
   
    
    
