function UAVGUI

% Create figure window
mainfig = uifigure;
mainfig.Name = "Crazyflie Flight Control";
mainfig.Position(3:4) = [1000 500];
movegui(mainfig,'center');

% Manage app layout
gl = uigridlayout(mainfig,[11 4]);
gl.RowHeight = {'1x','1x','1x','1x','1x','1x','1x','1x','1x','5x','1.5x'};
gl.ColumnWidth = {'1x','1x','0.2x','3x'};

% Radio Button
bg = uibuttongroup(gl);
rb1 = uiradiobutton(bg, "Position", [10 5 150 15],"Text", "PID Control");
rb2 = uiradiobutton(bg, "Position", [150 5 250 15], "Text", "Reinforcement Learning");
bg.Layout.Row = 2; bg.Layout.Column = [1 2];

% Create UI components
lbl1 = uilabel(gl,'Text','Reference Trajectory');
dd1 = uidropdown(gl);
dd1.Items = ["Select" "Helix" "Square" "Figure-8" "Straight Line" "Custom Coordinates"];
dd1.Value = "Select";
ax = uiaxes(gl);
btn = uibutton(gl,'Text','Simulate','BackgroundColor','#5D786F','FontColor','w');

% Lay out UI components
ax.Layout.Row = [1 11]; ax.Layout.Column = 4;
lbl1.Layout.Row = 4; lbl1.Layout.Column = 1;
dd1.Layout.Row = 4; dd1.Layout.Column = 2;
btn.Layout.Row = 11; btn.Layout.Column = [1 2];

% 3D plot
plot3(ax, NaN, NaN, NaN);
hold(ax,'on'); grid(ax,'on'); view(ax,-20,45);
xlim(ax, [-0.5 1.0]); ylim(ax, [-1 1.0]); zlim(ax, [-0.5 2.0]);
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');

% Helix and Figure-8
lbx = uilabel(gl,'Text','Width','Visible','off');
lby = uilabel(gl,'Text','Depth','Visible','off');
lbz = uilabel(gl,'Text','Height','Visible','off');
lbl = uilabel(gl,'Text','Curves','Visible','off');
lbx.Layout.Row = 6; lbx.Layout.Column = 1;
lby.Layout.Row = 7; lby.Layout.Column = 1;
lbz.Layout.Row = 8; lbz.Layout.Column = 1;
lbl.Layout.Row = 8; lbl.Layout.Column = 1;

slx = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
sly = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
slz = uislider(gl,'Limits',[0 0.1],'Value',0.1,'Visible','off');
sll = uislider(gl,'Limits',[0 10],'Value',1,'Visible','off');
slx.Layout.Row = 6; slx.Layout.Column = 2;
sly.Layout.Row = 7; sly.Layout.Column = 2;
slz.Layout.Row = 8; slz.Layout.Column = 2;
sll.Layout.Row = 8; sll.Layout.Column = 2;

% Square
lbs = uilabel(gl,'Text','Side','Visible','off');
lbs.Layout.Row = 6; lbs.Layout.Column = 1;
sls = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
sls.Layout.Row = 6; sls.Layout.Column = 2;

% Custom coordinates
lbi = uilabel(gl,'Text','Initial Position','Visible','off');
lbf = uilabel(gl,'Text','Final Position','Visible','off');
lbi.Layout.Row = 6; lbi.Layout.Column = 1;
lbf.Layout.Row = 6; lbf.Layout.Column = 2;

sla = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
slb = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
slc = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
sla.Layout.Row = 7; sla.Layout.Column = 1;
slb.Layout.Row = 8; slb.Layout.Column = 1;
slc.Layout.Row = 9; slc.Layout.Column = 1;

sld = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
sle = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
slf = uislider(gl,'Limits',[0 1],'Value',0.5,'Visible','off');
sld.Layout.Row = 7; sld.Layout.Column = 2;
sle.Layout.Row = 8; sle.Layout.Column = 2;
slf.Layout.Row = 9; slf.Layout.Column = 2;

ccf = uibutton(gl,'Text','Upload Custom Trajectory','Visible','off');
ccf.Layout.Row = 7; ccf.Layout.Column = [1 2];

% Assign callback function
dd1.ValueChangedFcn = {@changeTrajectory,lbx,lby,lbz,lbs,lbi,lbf,lbl,slx,sly,slz,sls,sla,slb,slc,sld,sle,slf,sll,ccf};
bg.SelectionChangedFcn = {@changeAlgorithm};
btn.ButtonPushedFcn = {@buttonCallback,rb1,rb2,dd1,ax,slx,sly,slz,sls,sla,sld,slb,sle,slc,slf,sll};
ccf.ButtonPushedFcn = {@buttonUpload};
end

%% --- Callback Functions --- %%

function changeTrajectory(src,event,lbx,lby,lbz,lbs,lbi,lbf,lbl,slx,sly,slz,sls,sla,slb,slc,sld,sle,slf,sll,ccf)
type = event.Value;
switch type
    case "Select"
        % nothing happens
    case "Helix"
        lbx.Visible = 'on';
        lby.Visible = 'on';
        lbz.Visible = 'on';
        slx.Visible = 'on';
        sly.Visible = 'on';
        slz.Visible = 'on';
        lbs.Visible = 'off';
        sls.Visible = 'off';
        lbi.Visible = 'off';
        lbf.Visible = 'off';
        sla.Visible = 'off';
        slb.Visible = 'off';
        slc.Visible = 'off';
        sld.Visible = 'off';
        sle.Visible = 'off';
        slf.Visible = 'off';
        ccf.Visible = 'off';
        sll.Visible = 'off';
        lbl.Visible = 'off';
    case "Square"
        lbs.Visible = 'on';
        sls.Visible = 'on';
        lbx.Visible = 'off';
        lby.Visible = 'off';
        lbz.Visible = 'off';
        slx.Visible = 'off';
        sly.Visible = 'off';
        slz.Visible = 'off';
        lbi.Visible = 'off';
        lbf.Visible = 'off';
        sla.Visible = 'off';
        slb.Visible = 'off';
        slc.Visible = 'off';
        sld.Visible = 'off';
        sle.Visible = 'off';
        slf.Visible = 'off';
        ccf.Visible = 'off';
        sll.Visible = 'off';
        lbl.Visible = 'off';
    case "Figure-8"
        lbx.Visible = 'on';
        lby.Visible = 'on';
        slx.Visible = 'on';
        sly.Visible = 'on';
        sll.Visible = 'off';
        lbl.Visible = 'off';
        lbs.Visible = 'off';
        sls.Visible = 'off';
        lbz.Visible = 'off';
        slz.Visible = 'off';
        lbi.Visible = 'off';
        lbf.Visible = 'off';
        sla.Visible = 'off';
        slb.Visible = 'off';
        slc.Visible = 'off';
        sld.Visible = 'off';
        sle.Visible = 'off';
        slf.Visible = 'off';
        ccf.Visible = 'off';
    case "Straight Line"
        lbi.Visible = 'on';
        lbf.Visible = 'on';
        sla.Visible = 'on';
        slb.Visible = 'on';
        slc.Visible = 'on';
        sld.Visible = 'on';
        sle.Visible = 'on';
        slf.Visible = 'on';
        lbs.Visible = 'off';
        sls.Visible = 'off';
        lbx.Visible = 'off';
        lby.Visible = 'off';
        lbz.Visible = 'off';
        slx.Visible = 'off';
        sly.Visible = 'off';
        slz.Visible = 'off';
        ccf.Visible = 'off';
        sll.Visible = 'off';
        lbl.Visible = 'off';
    case "Custom Coordinates"
        ccf.Visible = 'on';
        lbs.Visible = 'off';
        sls.Visible = 'off';
        lbx.Visible = 'off';
        lby.Visible = 'off';
        lbz.Visible = 'off';
        slx.Visible = 'off';
        sly.Visible = 'off';
        slz.Visible = 'off';
        lbi.Visible = 'off';
        lbf.Visible = 'off';
        sla.Visible = 'off';
        slb.Visible = 'off';
        slc.Visible = 'off';
        sld.Visible = 'off';
        sle.Visible = 'off';
        slf.Visible = 'off';
        sll.Visible = 'off';
        lbl.Visible = 'off';
end
end

function changeAlgorithm(src,event)
end

function buttonUpload(src,event,handles)
[filename, pathname] = uigetfile({'*.csv'},'File Selector');
if ~ischar(filename)
    return;  % User aborted the file selection
end
file = fullfile(pathname, filename);
Data = importdata(file);
handles.Data = Data;
guidata(src, handles);  % Store updated handles struct in the GUI
end

function buttonCallback(src,event,rb1,rb2,dd1,ax,slx,sly,slz,sls,sla,slb,slc,sld,sle,slf,sll,handles)
traj = dd1.Value;
rb1=rb1.Value;
rb2=rb2.Value;

xin = slx.Value; yin = sly.Value; zin = slz.Value;
side_in = sls.Value; sll = sll.Value;
cx1 = sla.Value; cy1 = slc.Value; cz1 = sle.Value;
cx2 = slb.Value; cy2 = sld.Value; cz2 = slf.Value;

% PID CONTROL
if rb1==true
    cla (ax, 'reset');

    t_end = 15; dt = 0.05; t = (0:dt:t_end)'; % Time Vector

    switch traj
        case "Select"
            x_d = NaN; y_d = NaN; z_d = NaN;
        case "Helix"
            x_d = xin - xin*cos(2*t); y_d = yin*sin(2*t); z_d = zin*t;
        case "Square"
            side = side_in;      
            z_d  = zeros(size(t));
            seg  = floor(4*(t/t_end)) + 1;
            frac = 4*(t/t_end) - (seg-1);
            x_d = zeros(size(t));  y_d = zeros(size(t));
            for k = 1:length(t)
                s = seg(k); f = frac(k);
                switch s
                    case 1, x_d(k)= side*f;    y_d(k)= 0;
                    case 2, x_d(k)= side;      y_d(k)= side*f;
                    case 3, x_d(k)= side*(1-f);y_d(k)= side;
                    case 4, x_d(k)= 0;         y_d(k)= side*(1-f);
                end
            end
        case "Figure-8"
            x_d = xin*sin(t); y_d = yin*sin(sll*t).*cos(t); z_d = zeros(size(t));
        case "Straight Line"
            seg  = floor(2*(t/t_end)) + 1;
            frac = 2*(t/t_end) - (seg-1);
            x_d = zeros(size(t));  y_d = zeros(size(t)); z_d  = zeros(size(t));
            for k = 1:length(t)
                s = seg(k); f = frac(k);
                switch s
                    case 1, x_d(k) = cx1 + (cx2-cx1)*f; y_d(k) = cy1 + (cy2-cy1)*f; z_d(k) = cz1 + (cz2-cz1)*f;
                    case 2, x_d(k) = cx1 + (cx2-cx1)*(1-f); y_d(k) = cy1 + (cy2-cy1)*(1-f); z_d(k) = cz1 + (cz2-cz1)*(1-f);
                end
            end
        case "Custom Coordinates"
            handles = guidata(src);
            [numRows,numCols] = size(handles.Data);
            N = numRows-1;
            seg  = floor(N*(t/t_end)) + 1;
            frac = N*(t/t_end) - (seg-1);
            x_d = zeros(size(t));  y_d = zeros(size(t)); z_d  = zeros(size(t));
            for k = 1:length(t)
                s = seg(k); f = frac(k);
                for row=1:N
                    x1 = handles.Data(row,1) ; y1 = handles.Data(row,2) ; z1 = handles.Data(row,3);
                    nrow = row+1;
                    x2 = handles.Data(nrow,1) ; y2 = handles.Data(nrow,2) ; z2 = handles.Data(nrow,3);
                    switch s
                        case row, x_d(k) = x1 + (x2-x1)*f; y_d(k) = y1 + (y2-y1)*f; z_d(k) = z1 + (z2-z1)*f;
                    end
                end
            end  
    end

    % GLOBAL REFERENCE
    global t_ref x_d_ref y_d_ref z_d_ref
    t_ref   = t;
    x_d_ref = x_d;
    y_d_ref = y_d;
    z_d_ref = z_d;

    % SIMULATE DYNAMICS
    options = odeset('AbsTol',1e-6,'RelTol',1e-6);
    q0      = zeros(22,1);
    [t_sim, q] = ode15s(@UAV, [0 t_end], q0, options);
    
    % Interpolate sim→uniform grid
    XYZ   = interp1(t_sim, q(:,1:3),   t);
    Euler = interp1(t_sim, q(:,4:6),   t);

    % SET UP TRAILS
    Tar = [x_d, y_d, z_d];
    trailSim = animatedline(ax,'Color','b','LineWidth',1.5);
    trailTar = animatedline(ax,'Color','m','LineStyle','--','LineWidth',1);
    
    hold (ax, 'on'); grid (ax, 'on'); view(ax, -20,45);
    xlim(ax, [-0.5 1.0]); ylim(ax, [-1 1.0]); zlim(ax, [-0.5 2.0]);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');

    % ANIMATE IN REAL-TIME
    armLen = 0.1;
    hArm1 = gobjects(1,1);
    hArm2 = gobjects(1,1);
    
    for k = 2:length(t)
        pos = XYZ(k,:)'; tar = Tar(k,:)'; ang = Euler(k,:);
        R   = matrixB2I(ang(1), ang(2), ang(3));
    
        addpoints(trailSim, pos(1), pos(2), pos(3));
        addpoints(trailTar, tar(1), tar(2), tar(3));
    
        % quad shape endpoints
        r1 = pos + R*[ armLen; 0; 0];
        r3 = pos + R*[-armLen; 0; 0];
        r2 = pos + R*[0; armLen; 0];
        r4 = pos + R*[0;-armLen; 0];
    
        delete([hArm1 hArm2]);
        hArm1 = plot3(ax,[r1(1) r3(1)], [r1(2) r3(2)], [r1(3) r3(3)], 'k-', 'LineWidth',2);
        hArm2 = plot3(ax,[r2(1) r4(1)], [r2(2) r4(2)], [r2(3) r4(3)], 'k-', 'LineWidth',2);
    
        drawnow;
        pause(dt);
    end

% REINFORCEMENT LEARNING
elseif rb2==true
    cla (ax, 'reset');

    t_end = 15; dt = 1.87; t = (0:dt:t_end)'; % Time Vector

    switch traj
        case "Select"
            x_d = NaN; y_d = NaN; z_d = NaN;
        case "Helix"
            x_d = xin - xin*cos(2*t); y_d = yin*sin(2*t); z_d = zin*t;
        case "Square"
            side = side_in;      
            z_d  = ones(size(t));
            seg  = floor(4*(t/t_end)) + 1;
            frac = 4*(t/t_end) - (seg-1);
            x_d = zeros(size(t));  y_d = zeros(size(t));
            for k = 1:length(t)
                s = seg(k); f = frac(k);
                switch s
                    case 1, x_d(k)= side*f;    y_d(k)= 0;
                    case 2, x_d(k)= side;      y_d(k)= side*f;
                    case 3, x_d(k)= side*(1-f);y_d(k)= side;
                    case 4, x_d(k)= 0;         y_d(k)= side*(1-f);
                end
            end
        case "Figure-8"
            x_d = xin*sin(t); y_d = yin*sin(sll*t).*cos(t); z_d = ones(size(t));
        case "Straight Line"
            seg  = floor(2*(t/t_end)) + 1;
            frac = 2*(t/t_end) - (seg-1);
            x_d = zeros(size(t));  y_d = zeros(size(t)); z_d  = zeros(size(t));
            for k = 1:length(t)
                s = seg(k); f = frac(k);
                switch s
                    case 1, x_d(k) = cx1 + (cx2-cx1)*f; y_d(k) = cy1 + (cy2-cy1)*f; z_d(k) = cz1 + (cz2-cz1)*f;
                    case 2, x_d(k) = cx1 + (cx2-cx1)*(1-f); y_d(k) = cy1 + (cy2-cy1)*(1-f); z_d(k) = cz1 + (cz2-cz1)*(1-f);
                end
            end
        case "Custom Coordinates"
            handles = guidata(src);
            [numRows,numCols] = size(handles.Data);
            t_end = length(numRows); dt = 0.05; t = (0:dt:t_end)'; % Time Vector
            x_d = handles.Data(:,1);
            y_d = handles.Data(:,2);
            z_d = handles.Data(:,3);
    end
    
    % WRITE CSV TO PYTHON
    T = [x_d(:), y_d(:), z_d(:)];
    writematrix(T, 'trajectory.csv');
    system('conda run -n drones python .\gym_pybullet_drones\examples\evaluate_waypoints.py --model_path .\gym_pybullet_drones\best_model\best_model.zip --read_csv True --gui True');

    
    % SET UP TRAILS
    Tar = [x_d, y_d, z_d];
        
    hold (ax, 'on'); grid (ax, 'on'); view(ax, -20,45);
    xlim(ax, [-0.5 1.0]); ylim(ax, [-1 1.0]); zlim(ax, [-0.5 2.0]);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    
    for k = 2:length(t)
        tar = Tar(k,:)';    
        scatter3(ax,tar(1),tar(2),tar(3));
        pause(dt);
    end

    % READ CSV FROM PYTHON
    RL = readmatrix('crazyflie_trajectory.csv','NumHeaderLines',1);
    rlx = RL(:,2); rly = RL(:,3); rlz = RL(:,4);

    RLTar = [rlx, rly, rlz];
    trailSim = animatedline(ax,'Color','b','LineWidth',1.5);

    % ANIMATE IN REAL-TIME
    armLen = 0.1;
    hArm1 = gobjects(1,1);
    hArm2 = gobjects(1,1);
    disp(RL(end,1));
    t_end = RL(end,1); dt = 0.1/3; t = (0:dt:t_end)'; % Time Vector

    for k = 2:length(t)
        rltar = RLTar(k,:)';
        addpoints(trailSim, rltar(1), rltar(2), rltar(3));

        % quad shape endpoints
        r1 = rltar + [ armLen; 0; 0];
        r3 = rltar + [-armLen; 0; 0];
        r2 = rltar + [0; armLen; 0];
        r4 = rltar + [0;-armLen; 0];
    
        delete([hArm1 hArm2]);
        hArm1 = plot3(ax,[r1(1) r3(1)], [r1(2) r3(2)], [r1(3) r3(3)], 'k-', 'LineWidth',2);
        hArm2 = plot3(ax,[r2(1) r4(1)], [r2(2) r4(2)], [r2(3) r4(3)], 'k-', 'LineWidth',2);
    
        drawnow;
        pause(dt);
    end
end

end

%% --- Supporting Functions: PID Control --- %%

function qdot = UAV(t,qmat)
    % Pull in the global reference arrays:
    global t_ref x_d_ref y_d_ref z_d_ref

    % 1) INTERPOLATE THE DESIRED
    x_d   = interp1(t_ref, x_d_ref, t);
    y_d   = interp1(t_ref, y_d_ref, t);
    z_d   = interp1(t_ref, z_d_ref, t);
    psi_d = 0;  % keep yaw ref at zero

    % 2) UNPACK STATES
    m   = 0.028;  jxx = 16.571e-6;  jyy = 16.656e-6;  jzz = 29.262e-6;  g = 9.81;
    x    = qmat(1);   y    = qmat(2);   z    = qmat(3);
    phi  = qmat(4);   theta= qmat(5);   psi  = qmat(6);
    u    = qmat(7);   v    = qmat(8);   w    = qmat(9);
    p    = qmat(10);  q    = qmat(11);  r    = qmat(12);
    zi   = qmat(13); phii = qmat(14); thetai = qmat(15); psii = qmat(16);
    ui   = qmat(17); vi   = qmat(18); wi     = qmat(19);
    pint = qmat(20); qi    = qmat(21); ri     = qmat(22);

    % 3) OUTER LOOPS (as before, but using the interpolated x_d,y_d,z_d)
    xB   =  cos(psi)*x + sin(psi)*y;
    yB   = -sin(psi)*x + cos(psi)*y;
    x_dB =  cos(psi)*x_d + sin(psi)*y_d;
    y_dB = -sin(psi)*x_d + cos(psi)*y_d;
    xe   = x_dB - xB;  ye = y_dB - yB;  ze = z_d - z;  psie = psi_d - psi;
    u_d = 2*xe;  v_d = 2*ye;  w_d = 2*ze + 0.5*zi;  r_d = 6*psie + psii - 0.35*r;
    u_d = sat(u_d,1); v_d= sat(v_d,1); w_d = sat(w_d,1);
    ue = u_d - u;  ve = v_d - v;  we = w_d - w;  re = r_d - r;
    phi_d   = -(ve + vi);     
    theta_d =  (ue + ui);
    U1      = 25*we + 15*wi + 0.27468;
    U4      = 120*re + 16.7*ri;
    phi_d   = sat(phi_d,20*pi/180);
    theta_d = sat(theta_d,20*pi/180);

    % 4) INNER LOOPS
    phie   = phi_d - phi;    thetae = theta_d - theta;
    p_d    = 6*phie   + 3*phii;      
    q_d    = 6*thetae + 3*thetai;
    pe     = p_d - p;        qe     = q_d - q;
    U2 = ( jxx*(250*pe + 500*pint) + 2.5*(jzz - jyy)*q*r ) / (jxx + 2.5);
    U3 = ( jyy*(250*qe + 500*qi)   + 2.5*(jxx - jzz)*p*r ) / (jyy + 2.5);

    % 5) PLANT DYNAMICS
    xd = w*(sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)) ...
       - v*(sin(psi)*cos(phi) - cos(psi)*sin(phi)*sin(theta)) ...
       + u*(cos(psi)*cos(theta));
    yd = v*(cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta)) ...
       - w*(cos(psi)*sin(phi) - sin(psi)*cos(phi)*sin(theta)) ...
       + u*(sin(psi)*cos(theta));
    zd = w*(cos(theta)*cos(phi)) - u*sin(theta) + v*(sin(phi)*cos(theta));
    phid   = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
    thetad = q*cos(phi) - r*sin(phi);
    psid   = r*(cos(phi)/cos(theta)) + q*(sin(phi)/cos(theta));
    ud = r*v - q*w + g*sin(theta);
    vd = p*w - r*u - g*sin(phi)*cos(theta);
    wd = q*u - p*v + U1/m - g*cos(theta)*cos(phi);
    pd = (-(jzz - jyy)*q*r + U2)/jxx;
    qd = (-(jxx - jzz)*p*r + U3)/jyy;
    rd = (-(jyy - jxx)*p*q + U4)/jzz;

    % 6) COLLECT
    qdot = [xd; yd; zd; phid; thetad; psid; ud; vd; wd; pd; qd; rd; ...
            ze; phie; thetae; psie; ue; ve; we; pe; qe; re];

end

function y = sat(u,limit)
    y = max(min(u,limit),-limit);
end

function R = matrixB2I(phi,theta,psi)
    Rz = [ cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1 ];
    Ry = [ cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta) ];
    Rx = [ 1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi) ];
    R  = Rz * Ry * Rx;
end