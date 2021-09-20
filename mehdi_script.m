clc;close all 
N = 500; Ts = 2*pi/N;
t = linspace(-pi, pi, N); 

x = 8*(sin(t)).^3; 
y = 8*(sin(2*t)).^3; 

vx = gradient(x, Ts);
vy = gradient(y, Ts);
ax = gradient(vx, Ts);
ay = gradient(vy, Ts);

%orientation
phi = atan2(vy, vx); 

%robot velocities
v = vx.*cos(phi) + vy.*sin(phi); 
omega = (vx.*ay - vy.*ax)./(vx.^2 + vy.^2); 

%Plotting
figure()
subplot(2, 1, 1)
plot(t,v, 'linewidth', 4) 
xlabel('time',  'FontSize', 14)
ylabel('velocity',  'FontSize', 14)
title('Linear velocity', 'FontSize', 18)
xlim([-pi  pi])

subplot(2, 1, 2)
plot(t,omega, 'linewidth', 4)
title('Angular velocity', 'FontSize', 18)
xlabel('time',  'FontSize', 14)
ylabel('velocity',  'FontSize', 14)
xlim([-pi  pi])
print -deps figures/task1_velocities 

figure()
h = figure; 
axis tight manual % this ensures that getframe() returns a consistent size
filename = 'figures/task1_trajectory.gif'; 
plot(x, y, 'b', 'linewidth', 3)
hold on 
for i=1:10:N      
    plot(x(1:i), y(1:i), 'g-', 'linewidth', 6)
    legend('Given Trajectory', "Robot's Path"); 
    drawnow; 
    
    %create GIF
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if i == 1
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else 
      imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end 

end
hold off


%task 2 
W = 1/2; r = 1/4; T=0.1; 

%initialize Inverse kinematics velocities
omega = zeros(1, N); v = zeros(1, N); 
vL = zeros(1, N); vR = zeros(1, N); 
omegaL = zeros(1, N); omegaR = zeros(1, N); 


% %initialize resulting forward kinematic variables: 
% x_f = zeros(1, N); y_f = zeros(1, N); phi_f = zeros(1, N); 

for n = 2:N-1
    %calculating inverse kinematics variables: 
    mu = 1/2*(sin(phi(n))*(y(n+1)-y(n))+cos(phi(n)*(x(n+1)-x(n))))...
        /(cos(phi(n))*(y(n+1)-y(n))-sin(phi(n))*(x(n+1)-x(n))); 
    x_m = (x(n)+x(n+1))/2; 
    y_m = (y(n)+y(n+1))/2; 
    
    x_star = x_m - mu/2 * (y(n+1) - y(n)); 
    y_star = y_m + mu/2 * (x(n+1)-x(n)); 
    
    R_n = sqrt((x(n) - x_star)^2 + (y(n)-y_star)^2); 
    theta_1 = atan2((y(n)-y_star), (x(n)-x_star)); 
    theta_2 = atan2((y(n+1)-y_star), (x(n+1)-x_star)); 
    del_phi = wrapToPi(theta_1 - theta_2); 
    
    %resulting Inv-Kinematics velocities: 
    omega(n) = del_phi/T; 
    v(n) = R_n*abs(omega(n));  
    vL(n) = (R_n-1/2 *W)*omega(n); 
    vR(n) = (R_n+1/2 *W)*omega(n); 
    omegaL(n) = vL(n)/r; 
    omegaR(n) = vR(n)/r; 
    
    %forward Kinematics: circular velocity motion model/Exact integration
    %model 
    x_f(n+1) = x_f(n) + (v(n)/omega(n))*(-sin(phi(n))+sin(phi(n)+omega(n)*T));
    y_f(n+1) = y_f(n) + (v(n)/omega(n))*(cos(phi(n))-cos(phi(n)+omega(n)*T));

end
figure()
plot(x_f,y_f)
figure()
subplot 221
plot(t,vL,'linewidth', 2)
xlabel('time',  'FontSize', 10)
ylabel('velocity',  'FontSize', 10)
title('Left Wheel velocity', 'FontSize', 14)
xlim([-pi  pi])

subplot 222
plot(t,vR, 'linewidth', 2)
xlabel('time',  'FontSize', 10)
ylabel('velocity',  'FontSize', 10)
title('Right Wheel velocity', 'FontSize', 14)
xlim([-pi  pi])

subplot 223
plot(t,omegaL, 'linewidth', 2)
xlabel('time',  'FontSize', 10)
ylabel('velocity',  'FontSize', 10)
xlim([-pi  pi])

title('Left Wheel angular velocity', 'FontSize', 14)
subplot 224
plot(t,omegaR, 'linewidth', 2)
xlabel('time',  'FontSize', 10)
ylabel('velocity',  'FontSize', 10)
title('Right Wheel angular velocity', 'FontSize', 14)
xlim([-pi  pi])

print -deps figures/task2



