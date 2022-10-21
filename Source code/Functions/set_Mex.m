function set_Mex(C_code_folder)

% set_Mex add the correct folder containing the mex functions to the path

% Input
% -- C_code_folder       : Folder containing the mex functions needed, depending on the location of the markers

start_path = fullfile(pwd, '/C code/');
allSubFolders = genpath(start_path);
remain = allSubFolders;
listOfFolderNames = {};
numberOfFolders = 0;
while true
	[singleSubFolder, remain] = strtok(remain, ':');
	if isempty(singleSubFolder)
        numberOfFolders = numberOfFolders - 1;
		break;
    end

    if (numberOfFolders > 0)

        listOfFolderNames = [listOfFolderNames singleSubFolder];
        addpath(listOfFolderNames{numberOfFolders});
    end

    numberOfFolders = numberOfFolders + 1;
end

% Target folder
target_folder = strcat(pwd, '/C code/', C_code_folder);

% Iterate through all the folders
for i = 1 : numberOfFolders

    if (strcmp(listOfFolderNames{i}, target_folder) == 1)
    
        addpath(listOfFolderNames{i});
    else

        rmpath(listOfFolderNames{i});
    end
end
end