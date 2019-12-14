clc; clear all; close all;
% Main plotting script for EE209AS project
% ========================================================================
% ========== 1. Settings
forgetFactor = 0.6;
movAvgExp = dsp.MovingAverage('Method','Exponential weighting', 'ForgettingFactor', forgetFactor);
ts2ep = 1/200; % timestep to episode
ep2hr = 10/3600;

% ========== 2. Select plots to plot together
ptype = 3; % 1: normal, 2: cart, 3: comparison
slideNum = 1;
[plotNames, titleInfo, legendInfo] = loadSlideInfo(slideNum, ptype);

% ========== 3. Read corresponding CSV files and store in data
data = {};
for idx = 1:length(plotNames)
    if ptype == 1
        csvpath = ['data/normal/pbtf_', plotNames{idx},'/run-pbtf_', plotNames{idx}, '_tb-tag-episode_reward_mean.csv'];
    elseif ptype == 2
        csvpath = ['data/cart/pbtf1_', plotNames{idx},'/run-pbtf1_', plotNames{idx}, '_tb-tag-episode_reward_mean.csv'];
    elseif ptype == 3
        csvpath = ['data/cart/ext_', plotNames{idx},'/run-ext_', plotNames{idx}, '_tb-tag-episode_reward_mean.csv'];
    end
    data{idx} = readtable(csvpath);
    data{idx}.Step = data{idx}.Step*ts2ep*ep2hr;
end
if ptype == 1
    plotNames{length(plotNames)+1} = 'baseline_pybullet';
    data{length(data)+1} = readtable('data/normal/baseline_pybullet_raw/run-baseline_pybullet_raw_tb-tag-episode_reward_mean.csv');
    data{length(data)}.Step = data{length(data)}.Step*ts2ep*ep2hr;
    % To ensure all recordings have equal amounts of data
    cutdx = find(data{end}.Step < 2.5); cutdx = cutdx(end);
elseif ptype == 2
    plotNames{length(plotNames)+1} = 'baseline_pybullet';
    data{length(data)+1} = readtable('data/cart/pybullet_baseline_cart/run-pybullet_baseline_cart_tb-tag-episode_reward_mean.csv');
    data{length(data)}.Step = data{length(data)}.Step*ts2ep*ep2hr;
    % To ensure all recordings have equal amounts of data
    cutdx = find(data{end}.Step < 3.5); cutdx = cutdx(end);
elseif ptype == 3
    cutdx = find(data{end}.Step < 2.5); cutdx = cutdx(end);
end


% ========== 4. Plot corresponding plots
cmap_full = hsv;
nSkip = floor(length(cmap_full)/length(plotNames));
cmap = cmap_full(1:nSkip:end, :);

figure(1); hold on; grid on;
% xlim([0, 1000*ep2hr]);
xlim([0, data{end}.Step(cutdx+1)]);
for idx = 1:length(plotNames)
    if (idx == length(plotNames))
        cmap(idx,:) = [0,0,0];
    end
    p_op = plot(data{idx}.Step, data{idx}.Value, 'Color', cmap(idx,:), 'LineWidth', 2.5);
    p_op.Color(4) = 0.3;
    plot(data{idx}.Step, movAvgExp(data{idx}.Value), 'Color', cmap(idx,:), 'LineWidth', 2.5);
end
title(titleInfo, 'FontSize', 15);
if slideNum == 99
    legend(legendInfo, 'Location', 'best', 'FontSize', 11);
else
    legend(legendInfo, 'Location', 'southeast', 'FontSize', 13);
end
xlabel('Elapsed Time [hours]', 'FontSize', 13);
ylabel('Rewards (max: 2000)', 'FontSize', 13);
ax = gca;
ax.FontSize = 13;

   
% ========== 5. Compute Transfer Learning metrics
tfm = {};
for idx = 1:length(plotNames)
    Jumpstart = data{idx}.Value(1) - data{end}.Value(1);
    AsympPerf = data{idx}.Value(55) - data{end}.Value(55);
    TotalRew = sum(data{idx}.Value(1:cutdx)) - sum(data{end}.Value(1:cutdx));
    TfRatio = sum(data{idx}.Value(1:cutdx))/sum(data{end}.Value(1:cutdx));
    minRew = min(min(data{idx}.Value(1:cutdx)), min(data{end}.Value(1:cutdx)));
    maxRew = max(max(data{idx}.Value(1:cutdx)), max(data{end}.Value(1:cutdx)));
    AdjTfRatio = (sum(data{idx}.Value(1:cutdx)) - minRew*cutdx) / (sum(data{end}.Value(1:cutdx)) - minRew*cutdx);
    
    tfm{idx} = table(Jumpstart, AsympPerf, TotalRew, TfRatio, AdjTfRatio);
    disp(plotNames{idx});
    disp(tfm{idx});
end
