classdef Maneuver < handle & dynamicprops
    %Maneuver Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Name
        Data
        FlightID
        StartTime
        EndTime
        RawDataSource
        ToolboxVersion
        Options
    end

    methods

        function obj = Maneuver(Name,Data)
            %Maneuver Construct an instance of this class by unique name
            obj.Name = Name;
            obj.Data = Data;
        end

    end % public methods

    methods(Static)
        
        function obj = loadobj(s)
            %loadobj
            %
            % If any of the saved properties no longer are defined in the
            % class definition, create a dynamic property and give a
            % warning.
            if isstruct(s)
                newObj = Maneuver(s.Name,s.Data); 
                f = fieldnames(s);
                for ii = 1:length(f)
                    if isprop(newObj,f{ii})
                        newObj.(f{ii}) = s.(f{ii});
                    else
                        addprop(newObj,f{ii});
                        newObj.(f{ii}) = s.(f{ii});
                        mc = metaclass(newObj);
                        warning(['Property ''' f{ii} ''' loaded into object'...
                            'as dynamic property.\nIt should be defined in '...
                            mc.Name '.m or its superclasses\nin order to '...
                            'maintain compatibility.'],[])
                    end
                end
                obj = newObj;
            else
                obj = s;
            end
        end

        function [ts,te] = SelectTimestamps(ulogFile)
            %SelectTimestamps Select timestamps from ULOG.
            
            % convert ulg to timetable
            disp('Loading ULOG...')
            data = ulg2tt(ulogFile);
            figure
            stackedplot(data(:,["NED_m","EulerAngles_rad","vb_m_s","omega_rad_s","input_rc"]))
            disp('Scroll to timeframe of interest. Press any key to continue...')
            pause
            ax = gca;
            tlim = ax.XLimits;
            clf
            tiledlayout(5,1)
            nexttile
            plot(data.Time,data.NED_m)
            xlim(tlim)
            legend('North [m]','East [m]','Down [m]')
            grid on
            grid minor
            box on
            nexttile
            plot(data.Time,data.EulerAngles_rad*180/pi)
            xlim(tlim)
            legend('Roll [deg]','Pitch [deg]','Yaw [deg]')
            grid on
            grid minor
            box on
            nexttile
            plot(data.Time,data.vb_m_s)
            xlim(tlim)
            legend('u [m/s]','v [m/s]','w [m/s]')
            grid on
            grid minor
            box on
            nexttile
            plot(data.Time,data.omega_rad_s*180/pi)
            xlim(tlim)
            legend('p [deg/s]','q [deg/s]','r [deg/s]')
            grid on
            box on
            nexttile
            plot(data.Time,data.input_rc)
            xlim(tlim)
            legend(num2str((1:size(data.input_rc,2))'))
            grid on
            grid minor
            box on
            disp('Select start of maneuver.')
            [tsd,~] = ginput(1);
            disp('Select end of maneuver.')
            [ted,~] = ginput(1);
            ts = seconds(tsd);
            te = seconds(ted);

        end % SelectTimestamps

        % Factory
        function obj = ProcessFlightData(filename,StartTime,EndTime)
            %ProcessFlightData Factory for ULOGs.
            
            % get maneuver, flight ID, and date from filename
            [~,FlightID,~] = fileparts(filename);

            % convert ulg to timetable
            disp('Processing ULOG...');
            opts = ulg2ttOptions;
            opts.MessageSet = 'SystemIdentification';
            tbl = ulg2tt(filename,opts);
            
            % loop through list of start and end times
            for ii = 1:length(StartTime)

                % trim to maneuver
                S = timerange(StartTime(ii),EndTime(ii));
                Data = tbl(S,:);
                
                % maneuver ID
                sStr = num2str(floor(seconds(StartTime(ii))));
                eStr = num2str(floor(seconds(EndTime(ii))));
                Name = [FlightID '_s' sStr '_e' eStr];
    
                % create maneuver object
                obj(ii) = Maneuver(Name,Data);
                obj(ii).FlightID = FlightID;
                obj(ii).RawDataSource = filename;
                obj(ii).StartTime = StartTime(ii);
                obj(ii).EndTime = EndTime(ii);

            end

        end % ProcessFlightData

    end % static methods

end % classdef