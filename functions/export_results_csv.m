function  export_results_csv(struct_to_open, variables, target_folder, file_name)

nvars = size(variables',1);

for k=1:nvars
    if k==1
        tiss = struct_to_open.(variables(k)) ;
    else
        tiss = [tiss, struct_to_open.(variables(k))];
    end
end

TT = timeseries2timetable( tiss );
TT = renamevars(TT,TT.Properties.VariableNames, variables);

writetimetable(TT, target_folder + file_name, 'Delimiter',',')

fprintf("Ok - File printed %s \n", target_folder + file_name)

end
