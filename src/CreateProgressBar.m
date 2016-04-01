function cpb = CreateProgressBar(maxVal)

% Create Instance 
cpb = ConsoleProgressBar();

% Set progress bar parameters 
cpb.setLeftMargin(3); % progress bar left margin 
cpb.setTopMargin(0); % rows margin

cpb.setLength(30); % progress bar length: [.....] 
cpb.setMinimum(0); % minimum value of progress range [min max] 
cpb.setMaximum(maxVal); % maximum value of progress range [min max]

cpb.setElapsedTimeVisible(1);
cpb.setRemainedTimeVisible(1);

cpb.setElapsedTimePosition('left');
cpb.setRemainedTimePosition('right');