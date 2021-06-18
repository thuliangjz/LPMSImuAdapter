function adapter_lib_gen(action)
    if strcmp(action, "gen")
        delete('defineLPMSAdapterLib.mlx')
        delete('defineLPMSAdapterLib.m')
        if ~exist('.\LPMSAdapterLib', 'dir')
            mkdir('.\LPMSAdapterLib')
        end
        projectPath = pwd();
        adapterPath = fullfile(projectPath, 'LPMSAdapter');
        adapterHeaderPath = fullfile(adapterPath, 'LPMSAdapter');
        adapterHeaderFile = fullfile(adapterHeaderPath, 'LPMSAdapter.h');
        libname = 'LPMSAdapterLib';
        includePaths = {adapterHeaderPath, fullfile(adapterPath, 'eigen'), fullfile(adapterPath, 'openzen', 'include')};
        libs = {fullfile(adapterPath, 'x64', 'Release', 'LPMSAdapter.lib'), fullfile(adapterPath, 'openzen', 'lib', 'x64', 'OpenZen.lib')};
        clibgen.generateLibraryDefinition(adapterHeaderFile, 'IncludePath', includePaths, 'Libraries', libs, ...
        'PackageName', libname, 'ReturnCArrays', false, 'Verbose', true);
        delete('defineLPMSAdapterLib.mlx')
        delete('defineLPMSAdapterLib.m')
        copyfile('scripts/defineLPMSAdapterLib_full.m', 'defineLPMSAdapterLib.m')
    end
    if strcmp(action, "build")
        build(defineLPMSAdapterLib);
        addLibs();
    end
    if strcmp(action, "add_lib")
        addLibs();
    end
    if strcmp(action, "clean")
        delete('defineLPMSAdapterLib.mlx', 'defineLPMSAdapterLib.m', 'LPMSAdapterLibData.xml');
    end
end

function addLibs()
    addpath('LPMSAdapterLib');
    adapterDllLoc = fullfile(pwd(), 'LPMSAdapter', 'x64', 'Release');
    openzenDllLoc = fullfile(pwd(), 'LPMSAdapter', 'openzen', 'lib', 'x64');
    syspath = getenv('PATH');
    setenv('PATH', [adapterDllLoc ';' openzenDllLoc ';' syspath]);
end