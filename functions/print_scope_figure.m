function [] = print_scope_figure(simulink_file, scope_name, destination_directory, file_name)

	%Open scope (necessary to have figure displayed)
	open_system(fullfile(simulink_file,scope_name))
	% Find the Scope (which is really just a figure window)
	hs = findall(0,'Name',scope_name);
	% Create a new target figure
	hf = figure('Position',get(hs,'Position'));
	% Get the handle to the panel containing the plots
	hp = findobj(hs.UserData.Parent,'Tag','VisualizationPanel');
	% Copy the panel to the new figure
	copyobj(hp,hf)
	% Change it to landscape orientation
	orient(hf,'landscape')
	% Print 
	print(hf, destination_directory + file_name ,'-dpdf', '-bestfit') 
	
end
