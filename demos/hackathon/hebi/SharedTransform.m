%SHAREDTRANSFORM enables sharing of a 4x4 transform across processes.
%   SHAREDTRANSFORM(name) provides access to a 4x4 transform that can be
%   shared across processes, e.g., across multiple MATLAB instances. This
%   is done using memory mapped files, which results in practically zero
%   overhead.
%   
%   Notes:
% 
%   -) Memory mapping requires a binary file backing the accessed memory.
%   The backing file is called '<name>.tmp' and is located in the directory
%   of this class. The backing file gets created by the first caller. The
%   file is deleted automatically once there are no more callers.
%
%   -) The initial value at the first file creation is the identity matrix
%
%   -) There can be multiple reader threads (and processes), but there
%   should only ever be a single writer thread.
%
%   -) Reads between threads are not guaranteed as MATLAB does not provide
%   access to memory barriers (e.g. volatile variables). There is also no
%   locking, so writes may intersect with reads.
%
%   -) The transform can be shared with processes in other languages. 
%   However, keep in mind that MATLAB stores matrices in column major 
%   format with potentially different endian-ness.
%
%   Example:
%      % MATLAB instance (writer)
%      blob = SharedTransform('blob');
%      blob.setTransform(eye(4));
%
%      % MATLAB instance (reader)
%      blob = SharedTransform('blob');
%      T = blob.getTransform();
%

%   Author: Florian Enner <florian @ hebirobotics.com>
%   Copyright 2015-2016 HEBI Robotics, LLC.
classdef SharedTransform < handle
    
    properties (SetAccess = private)
        store % path to the backing file
    end
    
    properties (Access = private)
        file
        fileLock
    end
    
    methods
        
        % constructor
        function this = SharedTransform(name)
            
            % find where this file is located
            scriptPath = which(mfilename());
            [path,~,~] = fileparts(scriptPath);
            
            % find the corresponding temporary file
            % in the directory of this script
            fileName = fullfile(path, [name '.tmp']);
            
            % make sure the file exists (create on first call)
            % (2 is magic number for file, not folder)
            if(exist(fileName, 'file') ~= 2)
                display(['creating backing file ' fileName]);
                
                % create file initialized to identity matrix
                fileID = fopen(fileName,'w');
                fwrite(fileID,eye(4),'double');
                fclose(fileID);
            end
            
            % memmap does not keep the file from being deleted, so we 
            % need to open the file manually in order to provide
            % auto-deletion.
            this.store = fileName;
            this.fileLock = fopen(this.store,'r');
            
            % make sure the fileSize is correct
            fileInfo = dir(fileName);
            if(fileInfo.bytes ~= 4*4*8)
                error(['Backing file has wrong dimensions. '...
                    'Please delete: ' fileName]);
            end
            
            % memory map the file
            this.file = memmapfile(fileName, ...
                'Format', {'double' [4 4] 'T';}, ...
                'Writable', true);
            
        end
        
        % destructor
        function delete(this)
            
            % unmap memory
            this.file = [];
            
            % give up lock and try to delete file. Note that we disable
            % warnings, as only the last one has permission to actually
            % delete the file.
            fclose(this.fileLock);
            warning('off', 'MATLAB:DELETE:Permission');
            delete(this.store);
            warning('on', 'MATLAB:DELETE:Permission');
            
        end
        
        function T = getTransform(this)
            T = this.file.Data.T(1:4,1:4); % force a copy
        end
        
        function R = getRotation(this)
            R = this.file.Data.T(1:3,1:3);
        end
        
        function p = getPosition(this)
            p = this.file.Data.T(1:3,4);
        end
        
        function [] = setTransform(this, T)
            this.file.Data.T(1:4,1:4) = T;
        end
        
        function [] = setRotation(this, R)
            this.file.Data.T(1:3,1:3) = R;
        end
        
        function [] = setPosition(this, p)
            this.file.Data.T(1:3,4) = p;
        end
        
    end
end
