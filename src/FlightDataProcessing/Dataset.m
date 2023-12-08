classdef Dataset < handle & dynamicprops
    %Dataset Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Name
        Data
        Domain
        ToolboxVersion
    end % properties

    methods
        
        function obj = Dataset(Name)
            %Dataset Construct an instance of this class

            % Required model properties
            obj.Name = Name;

            % initialize dataset map
            obj.Data = containers.Map; 

        end % Dataset

        function AddManeuver(obj,ManeuverList)
            for ii = 1:length(ManeuverList)
                obj.Data(ManeuverList(ii).Name) = ManeuverList(ii);
            end
        end % AddManeuver

        function [data,datai,Ni] = GetAllData(obj,VariableNames)
            M = values(obj.Data);
            N = 0;
            Ni = zeros(length(M)+1,1);
            for ii = 1:length(M)
                Ni(ii+1,1) = N + height(M{ii}.Data);
                N = N + height(M{ii}.Data);
            end
            for ii = 1:length(M)
                for jj = 1:length(VariableNames)
                    data.(VariableNames{jj})(Ni(ii)+1:Ni(ii+1),:) = M{ii}.Data.(VariableNames{jj});
                    datai(ii,1).(VariableNames{jj}) = M{ii}.Data.(VariableNames{jj});
                end
            end
        end % GetAllData

        function [data,datai,Ni] = GetData(obj,ManeuverNames,VariableNames)
            N = 0;
            Ni = zeros(length(ManeuverNames)+1,1);
            M = cell(length(ManeuverNames),1);
            for ii = 1:length(ManeuverNames)
                M{ii} = obj.Data(ManeuverNames{ii});
                Ni(ii+1,1) = N + height(M{ii}.Data);
                N = N + height(M{ii}.Data);
            end
            for ii = 1:length(M)
                for jj = 1:length(VariableNames)
                    data.(VariableNames{jj})(Ni(ii)+1:Ni(ii+1),:) = M{ii}.Data.(VariableNames{jj});
                    datai(ii,1).(VariableNames{jj}) = M{ii}.Data.(VariableNames{jj});
                end
            end
        end % GetAllData

        function ComputeDomain(obj,VariableNames)
            data = obj.GetAllData(VariableNames);
            for ii = 1:length(VariableNames)
                obj.Domain.(VariableNames{ii}).Mean = mean(data.(VariableNames{ii}));
                obj.Domain.(VariableNames{ii}).Minimum = min(data.(VariableNames{ii}));
                obj.Domain.(VariableNames{ii}).Maximum = max(data.(VariableNames{ii}));
                obj.Domain.(VariableNames{ii}).StandardDeviation = std(data.(VariableNames{ii}));
                obj.Domain.(VariableNames{ii}).Median = median(data.(VariableNames{ii}));
            end
        end % ComputeDomain

    end % public methods

    methods(Access = protected)
        % Nothing so far
    end % protected methods

    methods(Static)
        
        function obj = loadobj(s)
            %loadobj
            %
            % If any of the saved properties no longer are defined in the
            % class definition, create a dynamic property and give a
            % warning.
            if isstruct(s)
                newObj = Dataset(s.Name); 
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

    end % static methods

end % classdef