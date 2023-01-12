classdef ulg2ttOptions
    %ulg2ttOptions
    %
    % This class defines the options used in the ulg2tt function.
    %
    % The MessageSet property defines set of messages that are returned in
    % a timetable. The options are 'Debugging', 'Basic', 'Standard',
    % 'Advanced', and 'Estimation', although more may be added.
    %
    % The TimeStep property is specified as a numeric value in seconds.
    % However, the TimeStep property is ignored if the MessageSet is
    % 'Estimation'.
    %
    % The Plot propery is either a logical value indicating whether plots
    % are to be produced or a specific variabe name in the resulting
    % timetable, such as 'EulerAngles_rad'.
    %
    % The SaveToMAT property determines whether the data will be saved to a
    % MAT file in the current working directory.
    %

    properties
        TimeStep
        MessageSet
        Plot
        SaveToMAT
    end

    methods
        function obj = ulg2ttOptions()
            %ulg2ttOptions
            % Construct an instance of this class. Populate default values
            % for the properties.
            obj.TimeStep = 0.1;
            obj.MessageSet = 'Standard';
            obj.Plot = false;
            obj.SaveToMAT = false;
        end
    end
end