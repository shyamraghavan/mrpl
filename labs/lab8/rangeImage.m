classdef rangeImage < handle
    %rangeImage Stores a 1D range image and provides related services.

    properties(Constant)
        maxUsefulRange = 2.0;       
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
    end

    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
        clean;
    end
    
    methods(Access = public)
        
        function obj = rangeImage(ranges,skip,cleanFlag)
        % Constructs a rangeImage for the supplied data. 
        % Converts the data to rectangular coordinates
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                
                obj.numPix = n;
                obj.clean = false;
                if cleanFlag; 
                    obj.removeBadPoints();
                end;
            end
        end

        function removeBadPoints(obj)
        % takes all points above and below two range thresholds 
        % out of the arrays. This is a convenience but the result 
        % should not be used by any routine that expects the points 
        % to be equally separated in angle. The operation is done 
        % inline and removed data is deleted. 
            n = 0;
            cleanRArray = zeros(1,360);
            cleanTArray = zeros(1,360);
            cleanXArray = zeros(1,360);
            cleanYArray = zeros(1,360);
            for i = 1:obj.numPix
                if obj.rArray(i) > obj.minUsefulRange && obj.rArray(i) < obj.maxUsefulRange
                    n = n+1;
                    cleanRArray(n) = obj.rArray(i);
                    cleanTArray(n) = obj.tArray(i);
                    cleanXArray(n) = obj.xArray(i);
                    cleanYArray(n) = obj.yArray(i);
                end
            end
            obj.numPix= n;
            obj.rArray = cleanRArray(1:n);
            obj.tArray = cleanTArray(1:n);
            obj.xArray = cleanXArray(1:n);
            obj.yArray = cleanYArray(1:n);
            obj.clean = true;
        end

        function plotRvsTh(obj, maxRange)
        % plot the range image after removing all points exceeding
        % maxRange 
        
            %%% FILL ME IN %%%
        end

        function plotXvsY(obj, handle)
        % plot the range image after removing all points exceeding
        % maxRange 
            set(handle,'XDATA',obj.xArray,...
                       'YDATA',obj.yArray,...
                       'Color','b',...
                       'LineStyle','none',...
                       'Marker','+');
                       xlim([-2,2]);
                       ylim([-2,2]);
                       hold on;
        end

        function [err, num, th] = findLineCandidate(obj,middle,maxLen)
        % Find the longest sequence of pixels centered at pixel 
        % “middle” whose endpoints are separated by a length less 
        % than the provided maximum. Return the line fit error, the 
        % number of pixels participating, and the angle of 
        % the line relative to the sensor. 
            len = 0;
            num = 1;
            maxAngle = ceil(atan(maxLen/(2*obj.minUsefulRange))*(180/pi));
%             maxAngle = 150;
            pArray = zeros(2,2*maxAngle+1);
            pArray(:,maxAngle)=[obj.xArray(middle);obj.yArray(middle)];
            
            while  len < maxLen && num < maxAngle
                pLx = obj.xArray(indexAdd(obj,middle,+num));
                pLy = obj.yArray(indexAdd(obj,middle,+num));
                pArray(:,maxAngle-num)=[pLx;pLy];
                pRx = obj.xArray(indexAdd(obj,middle,-num));
                pRy = obj.yArray(indexAdd(obj,middle,-num));
                pArray(:,maxAngle+num)=[pRx;pRy];
                
                len = norm([pRx-pLx,pRy-pLy]);
                num = num+1;
                
            end
            % chop off the ends that are not on our line
            num = num -2;
            pArray = pArray(:,maxAngle-num:maxAngle+num);
            center = ceil(length(pArray)/2);
            
            % [a,b] for proposed line
            if pRx ~= pLx %general case where there is a slope.
                a = (pRy-pLy)/(pRx-pLx);
                b = pLy-a*pLx;
                % matrix to calculate the perpendicular intersect from a given
                % point to our line.
                T = [1/(1+a^2) , -a/(1+a^2);...
                     a/(1+a^2) , a^2/(1+a^2)];
                % create a collumn matrix of intercept points
                linArray = pArray'*T;
                linArray(:,1) = linArray(:,1)-b*(a/(1+a^2));
                linArray(:,2) = linArray(:,2)+b*(1-a/(1+a^2));

                % sum the distances between the points in pArray and their
                % respective points on our line in linArray.
                err = sum(sqrt(sum((pArray-linArray').^2,1)),2);

                % the angle of the line
                if (pArray(1,center)*(a-1)-a*pArray(2,center)) ~= 0
                    th = atan2((a*pArray(2,center)),...
                               (pArray(1,center)*(a-1)-a*pArray(2,center)));
                else
                    th = pi/2;
                end
            else % case where the slope is infinite
                if pRx > 0
                    th = 0;
                else
                    th = pi;
                end
                err = sum(abs(pRx-pArray),2);
                err = err(1);
            end
        end
        
        function num = numPixels(obj)
            num = obj.numPix;
        end
        
        function [x,y]=getTarget(obj,i,th,offset)
            deltaX = offset*cos(th);
            deltaY = offset*sin(th);
            x = obj.xArray(i)-deltaX;
            y = obj.yArray(i)-deltaY;
        end     
        
        % Modulo arithmetic on nonnegative integers. MATLABs choice to
        % have matrix indices start at 1 has implications for 
        % calculations relating to the position of a number in the 
        % matrix. In particular, if you want an operation defined on 
        % the sequence of numbers 1 2 3 4 that wraps around, the 
        % traditional modulus operations will not work correctly. 
        % The trick is to convert the index to 0 1 2 3 4, do the 
        % math, and convert back. 

        function out = inc(obj,in)
        % increment with wraparound over natural numbers 
            out = indexAdd(obj,in,1);
        end

        function out = dec(obj,in)
        % decrement with wraparound over natural numbers
            out = indexAdd(obj,in,-1);
        end

        function out = indexAdd(obj,a,b)
        % add with wraparound over natural numbers. First number 
        % “a” is "natural" meaning it >=1. Second number is signed. 
        % Convert a to 0:3 and add b (which is already 0:3). 
        % Convert the result back by adding 1.
            out = mod((a-1)+b,obj.numPix)+1;
        end
    end
end
