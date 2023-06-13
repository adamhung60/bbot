x = 100;
%y = -226.8;
y = -223;
L1 = 136.263;
L2 = 260;
dir = 1;
q2 = dir* acosd((x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2));
q1 = atand(y/x) + atand((L2*sind(q2))/(L1+L2*cosd(q2)));


l1endx = L1*cosd(q1);
l1endy = L1*sind(q1);

l2endx = l1endx+L2*cosd(q1-q2);
l2endy = l1endy+L2*sind(q1-q2);

hold on;

plot(0,0,'.','MarkerSize', 50);
plot([0 l1endx], [0 l1endy], 'b','LineWidth',10);
plot([l1endx l2endx], [l1endy l2endy], 'r','LineWidth',10);
plot([-100 200],[y y],'k','LineWidth',5);

xlim([-100 200]);
ylim([-300 100]);


q1
q2