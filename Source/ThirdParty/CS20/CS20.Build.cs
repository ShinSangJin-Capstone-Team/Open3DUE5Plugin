// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class CS20 : ModuleRules
{
	public CS20(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
        bUseRTTI = true;

        if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			// Add the import library
			PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "libsynexens3.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_calib3d440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_core440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_features2d440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_flann440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_highgui440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_imgcodecs440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_imgproc440.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "shared", "opencv_videoio440.lib"));

            // Add any include paths for the plugin
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include"));
            PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "include", "opencv2"));

            // Ensure that the DLL is staged along with the executable
            /**/
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/csreconstruction.dll");
            //RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/libsynexens3.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_calib3d440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_core440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_features2d440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_flann440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_highgui440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_imgcodecs440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_imgproc440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/opencv_videoio440.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/pthreadVC2.dll");
            RuntimeDependencies.Add("$(PluginDir)/Binaries/ThirdParty/CS20/bin/SonixCamera.dll");

            // Delay-load the DLL, so we can load it from the right place first
            /**/
            PublicDelayLoadDLLs.Add("csreconstruction.dll");
            PublicDelayLoadDLLs.Add("opencv_calib3d440.dll");
            PublicDelayLoadDLLs.Add("opencv_features2d440.dll");
            PublicDelayLoadDLLs.Add("opencv_flann440.dll");
            PublicDelayLoadDLLs.Add("opencv_core440.dll");
            PublicDelayLoadDLLs.Add("opencv_highgui440.dll");
            PublicDelayLoadDLLs.Add("opencv_imgcodecs440.dll");
            PublicDelayLoadDLLs.Add("opencv_imgproc440.dll");
            PublicDelayLoadDLLs.Add("opencv_videoio440.dll");
            //PublicDelayLoadDLLs.Add("libsynexens3.dll");
            PublicDelayLoadDLLs.Add("pthreadVC2.dll");
            PublicDelayLoadDLLs.Add("SonixCamera.dll");
            //*/
        }
		/**
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            PublicDelayLoadDLLs.Add(Path.Combine(ModuleDirectory, "Mac", "Release", "libExampleLibrary.dylib"));
            RuntimeDependencies.Add("$(PluginDir)/Source/ThirdParty/Open3D/Mac/Release/libExampleLibrary.dylib");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			string ExampleSoPath = Path.Combine("$(PluginDir)", "Binaries", "ThirdParty", "Open3D", "Linux", "x86_64-unknown-linux-gnu", "libExampleLibrary.so");
			PublicAdditionalLibraries.Add(ExampleSoPath);
			PublicDelayLoadDLLs.Add(ExampleSoPath);
			RuntimeDependencies.Add(ExampleSoPath);
		}
		//*/
	}
}
