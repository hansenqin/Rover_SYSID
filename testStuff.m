u = []; v = []; r = [];
for i = 1:size(state)
    u = [u, state(i).U / 10];
    v = [v, state(i).V];
    r = [r, state(i).R];
end

u_unsmoothed = [];

for i = 1:size(unsmoothed_speed)
    u_unsmoothed = [u_unsmoothed, unsmoothed_speed(i).Data];
end

X = [];
Y = [];
for i = 1:size(tf)
    if size(tf(i).Transforms, 2) > 0
        if tf(i).Transforms.Header.FrameId == "map" && tf(i).Transforms.ChildFrameId == "velodyne_base_link"
            X = [X, tf(i).Transforms.Transform.Translation.X];
            Y = [Y, tf(i).Transforms.Transform.Translation.Y];
        end
    end
end

I = [];

for i = 1:size(current)
    I = [I, current(i).Data];
end

hold on;
% plot(u, 'r');
% plot(I, 'b');
plot(X, Y);
hold off;