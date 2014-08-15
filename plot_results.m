% Plot distribution of path lengths
hold all;
% [f,xi] = ksdensity(lengths0);
% plot(xi,f,'LineWidth',3);
[f,xi] = ksdensity(lengths1);
plot(xi,f,'LineWidth',3);
[f,xi] = ksdensity(lengths2);
plot(xi,f,'LineWidth',3);
[f,xi] = ksdensity(lengths3);
plot(xi,f,'LineWidth',3);
[f,xi] = ksdensity(lengths4);
plot(xi,f,'LineWidth',3);
xlim([0 800]);
h = legend('Adaptive','No Orient','Discrete','Discrete, Exponentially Weighted');
% h = legend('Basic','Adaptive','No Orient','Discrete','Discrete, Exponentially Weighted');
set(h,'FontSize',20);
xlabel('Path Length (Iterations)','FontSize',20);
ylabel('Probability','FontSize',20);
title('Distribution of Path Lengths for First Swarm under Each Model','FontSize',20);

% Plot path lengths over time
hold all;
% m = mean(dists0(:,1:1000));
% plot(m,'LineWidth',3);
m = mean(dists1(:,1:1000));
plot(m,'LineWidth',3);
m = mean(dists2(:,1:1000));
plot(m,'LineWidth',3);
m = mean(dists3(:,1:1000));
plot(m,'LineWidth',3);
m = mean(dists4(:,1:1000));
plot(m,'LineWidth',3);
xlim([0 200]);
% h = legend('Basic','Adaptive','No Orient','Discrete','Discrete, Exponentially Weighted');
h = legend('Adaptive','No Orient','Discrete','Discrete, Exponentially Weighted');
set(h,'FontSize',20);
xlabel('Iteration','FontSize',20);
ylabel('Distance from food source','FontSize',20);
title('Mean Distance from Food Source over Time','FontSize',20);


% Plot length distribution means over swarm (size with error bars)
hold all;
num_trials = 120;
for i = 2:5
  errorbar(ns(2:5), m(2:5,i),s(2:5,i)/sqrt(num_trials));
end
legend('Adaptive','No Orient','Discrete','Discrete, Exponentially Weighted');