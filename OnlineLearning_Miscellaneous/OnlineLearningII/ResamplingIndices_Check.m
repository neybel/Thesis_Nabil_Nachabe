
%%%% You should habe the resample_indices array in the workspace to do this
%%%% check!!!
% Determine the range of indices
num_particles = max(resample_indices);

% Check for omitted indices
all_indices = 1:num_particles;
omitted_indices = setdiff(all_indices, resample_indices);

% Check for replicated indices
[unique_indices, ~, idx] = unique(resample_indices);
counts = histc(idx, unique(idx));
replicated_indices = unique_indices(counts > 1);

% Display results
disp('Omitted Indices:');
disp(omitted_indices);

disp('Replicated Indices:');
disp(replicated_indices);