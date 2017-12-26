/*
Copyright (c) 2017 Ryoichi Ishikawa. All rights reserved.

This file contains Original Code and/or Modifications of Original Code
This software is released under the MIT License.
http://opensource.org/licenses/mit-license.php
*/


//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "pch.h"
#include "HolographicSpatialMappingMain.h"
#include "Common\DirectXHelper.h"
#include "Eigen\Eigen"
#include "Eigen\Core"

#include <windows.graphics.directx.direct3d11.interop.h>
#include <Collection.h>

#define HOSTNAME "localhost"
#define PORT "1234"
#define PORT2 "12034"
#define PORT3 "12340"
#define HEADER_CAMERA 10
#define HEADER_IMAGE 20
#define HEADER_CHANC 30
#define HEADER_POSLOST 40
#define HEADER_TRACKLOST 50
#define HOLOLENS_HEIGHT 60
#define HEADER_DEPTHSTREAM 100

#define CMD_IMG_REQ 4
#define FINISH_ALIGNMENT 8
#define LOC_FAILED 12
#define DEPTH_STREAM_REQ 16
#define DEPTH_STREAM_OFF_REQ 18
#define FINISH_RECV_MESH 20

using namespace HolographicSpatialMapping;
using namespace WindowsHolographicCodeSamples;

using namespace concurrency;
using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::Foundation::Collections;
using namespace Windows::Foundation::Numerics;
using namespace Windows::Graphics::DirectX;
using namespace Windows::Graphics::Holographic;
using namespace Windows::Perception::Spatial;
using namespace Windows::Perception::Spatial::Surfaces;
using namespace Windows::UI::Input::Spatial;
using namespace Windows::Networking;
using namespace Windows::Media::Capture;
using namespace Windows::Media::Render;
using namespace Windows::Media::MediaProperties;
using namespace std::placeholders;


Windows::Storage::Streams::DataWriter^							 writer;
Windows::Storage::Streams::DataReader^							reader;
Windows::Networking::Sockets::StreamSocket^					sock, ^imagesock;
bool ack = false, ack2 = false, ack3 = false,
posfailed = false, m_depthStreamMode = false,
checkReceive = true, connecting = false, height_requested = true;

//receive command sent from ROS
void receiveLoop(Windows::Storage::Streams::DataReader^ reader_, Windows::Networking::Sockets::StreamSocket^ sock_) {
	if (reader_ == nullptr)return;
	create_task(reader->LoadAsync(sizeof(UINT32))).then([reader_, sock_](unsigned int size)
	{
		if (size < sizeof(UINT32))
		{
			// The underlying socket was closed before we were able to read the whole data.
			cancel_current_task();
		}
		unsigned int stringLength = reader->ReadUInt32();
		if (stringLength == CMD_IMG_REQ)ack = true;
		if (stringLength == FINISH_ALIGNMENT)ack2 = true;
		if (stringLength == LOC_FAILED)posfailed = true;
		if (stringLength == DEPTH_STREAM_REQ)m_depthStreamMode = true;
		if (stringLength == DEPTH_STREAM_OFF_REQ)m_depthStreamMode = false;
		if (stringLength == FINISH_RECV_MESH)checkReceive = true;
		receiveLoop(reader_, sock_);
	});
}

//conenction callback
void OnEvent(Windows::Networking::Sockets::StreamSocketListener^ listener,
	Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs^ object) {
	writer = ref new Windows::Storage::Streams::DataWriter(object->Socket->OutputStream);
	reader = ref new Windows::Storage::Streams::DataReader(object->Socket->InputStream);
	sock = object->Socket;
	connecting = true;
	receiveLoop(reader, sock);
};


// Loads and initializes application assets when the application is loaded.
HolographicSpatialMappingMain::HolographicSpatialMappingMain(
    const std::shared_ptr<DX::DeviceResources>& deviceResources) :
    m_deviceResources(deviceResources)
{
    // Register to be notified if the device is lost or recreated.
    m_deviceResources->RegisterDeviceNotify(this);
	//Setting TCP connection listener
	HostName^ hostName;
	try
	{
		hostName = ref new HostName(HOSTNAME);
	}
	catch (InvalidArgumentException^ e)
	{
		return;
	}
	
	listener = ref new Sockets::StreamSocketListener();
	OnConnection = ref new Windows::Foundation::TypedEventHandler<Windows::Networking::Sockets::StreamSocketListener^, Windows::Networking::Sockets::StreamSocketListenerConnectionReceivedEventArgs^>
		(OnEvent);

	listener->ConnectionReceived += OnConnection;

	listener->Control->KeepAlive = false;
	create_task(listener->BindServiceNameAsync(PORT)).then([this](task<void> previousTask)
	{
		try
		{
			// Try getting an exception.
			previousTask.get();

		}
		catch (Exception^ exception)
		{
		}
	});
}

void HolographicSpatialMappingMain::SetHolographicSpace(
    HolographicSpace^ holographicSpace)
{
    UnregisterHolographicEventHandlers();

    m_holographicSpace = holographicSpace;

#ifdef DRAW_SAMPLE_CONTENT
    // Initialize the sample hologram.
    m_meshRenderer = std::make_unique<RealtimeSurfaceMeshRenderer>(m_deviceResources);

    m_spatialInputHandler = std::make_unique<SpatialInputHandler>();
#endif

    // Use the default SpatialLocator to track the motion of the device.
    m_locator = SpatialLocator::GetDefault();

	//This callback called when HoloLens lost its position
	m_locatabilityChangedToken =
		m_locator->LocatabilityChanged +=
		ref new Windows::Foundation::TypedEventHandler<SpatialLocator^, Object^>(
			std::bind(&HolographicSpatialMappingMain::OnLocatabilityChanged, this, _1, _2)
			);

    // This sample responds to changes in the positional tracking state by cancelling deactivation 
    // of positional tracking.
    m_positionalTrackingDeactivatingToken =
        m_locator->PositionalTrackingDeactivating +=
            ref new Windows::Foundation::TypedEventHandler<SpatialLocator^, SpatialLocatorPositionalTrackingDeactivatingEventArgs^>(
                std::bind(&HolographicSpatialMappingMain::OnPositionalTrackingDeactivating, this, _1, _2)
                );
            

    // Respond to camera added events by creating any resources that are specific
    // to that camera, such as the back buffer render target view.
    // When we add an event handler for CameraAdded, the API layer will avoid putting
    // the new camera in new HolographicFrames until we complete the deferral we created
    // for that handler, or return from the handler without creating a deferral. This
    // allows the app to take more than one frame to finish creating resources and
    // loading assets for the new holographic camera.
    // This function should be registered before the app creates any HolographicFrames.
    m_cameraAddedToken =
        m_holographicSpace->CameraAdded +=
            ref new Windows::Foundation::TypedEventHandler<HolographicSpace^, HolographicSpaceCameraAddedEventArgs^>(
                std::bind(&HolographicSpatialMappingMain::OnCameraAdded, this, _1, _2)
                );

    // Respond to camera removed events by releasing resources that were created for that
    // camera.
    // When the app receives a CameraRemoved event, it releases all references to the back
    // buffer right away. This includes render target views, Direct2D target bitmaps, and so on.
    // The app must also ensure that the back buffer is not attached as a render target, as
    // shown in DeviceResources::ReleaseResourcesForBackBuffer.
    m_cameraRemovedToken =
        m_holographicSpace->CameraRemoved +=
            ref new Windows::Foundation::TypedEventHandler<HolographicSpace^, HolographicSpaceCameraRemovedEventArgs^>(
                std::bind(&HolographicSpatialMappingMain::OnCameraRemoved, this, _1, _2)
                );

    // This code sample uses a DeviceAttachedFrameOfReference to have the Spatial Mapping surface observer
    // follow along with the device's location.
    m_referenceFrame = m_locator->CreateAttachedFrameOfReferenceAtCurrentHeading();
	m_stationaryReferenceFrame = m_locator->CreateStationaryFrameOfReferenceAtCurrentLocation();

    // Notes on spatial tracking APIs:
    // * Stationary reference frames are designed to provide a best-fit position relative to the
    //   overall space. Individual positions within that reference frame are allowed to drift slightly
    //   as the device learns more about the environment.
    // * When precise placement of individual holograms is required, a SpatialAnchor should be used to
    //   anchor the individual hologram to a position in the real world - for example, a point the user
    //   indicates to be of special interest. Anchor positions do not drift, but can be corrected; the
    //   anchor will use the corrected position starting in the next frame after the correction has
    //   occurred.
}

void HolographicSpatialMappingMain::UnregisterHolographicEventHandlers()
{
    if (m_holographicSpace != nullptr)
    {
        // Clear previous event registrations.

        if (m_cameraAddedToken.Value != 0)
        {
            m_holographicSpace->CameraAdded -= m_cameraAddedToken;
            m_cameraAddedToken.Value = 0;
        }

        if (m_cameraRemovedToken.Value != 0)
        {
            m_holographicSpace->CameraRemoved -= m_cameraRemovedToken;
            m_cameraRemovedToken.Value = 0;
        }
    }

    if (m_locator != nullptr)
    {
        m_locator->PositionalTrackingDeactivating -= m_positionalTrackingDeactivatingToken;
    }

    if (m_surfaceObserver != nullptr)
    {
        m_surfaceObserver->ObservedSurfacesChanged -= m_surfacesChangedToken;
    }
}

HolographicSpatialMappingMain::~HolographicSpatialMappingMain()
{
    // Deregister device notification.
    m_deviceResources->RegisterDeviceNotify(nullptr);

    UnregisterHolographicEventHandlers();
}

void HolographicSpatialMappingMain::OnSurfacesChanged(
    SpatialSurfaceObserver^ sender, 
    Object^ args)
{
    IMapView<Guid, SpatialSurfaceInfo^>^ const& surfaceCollection = sender->GetObservedSurfaces();

    // Process surface adds and updates.
    for (const auto& pair : surfaceCollection)
    {
        auto id = pair->Key;
        auto surfaceInfo = pair->Value;

        // Choose whether to add, or update the surface.
        // In this example, new surfaces are treated differently by highlighting them in a different
        // color. This allows you to observe changes in the spatial map that are due to new meshes,
        // as opposed to mesh updates.
        // In your app, you might choose to process added surfaces differently than updated
        // surfaces. For example, you might prioritize processing of added surfaces, and
        // defer processing of updates to existing surfaces.
        if (m_meshRenderer->HasSurface(id))
        {
            if (m_meshRenderer->GetLastUpdateTime(id).UniversalTime < surfaceInfo->UpdateTime.UniversalTime)
            {
                // Update existing surface.
                m_meshRenderer->UpdateSurface(id, surfaceInfo);
            }
        }
        else
        {
            // New surface.
            m_meshRenderer->AddSurface(id, surfaceInfo);
        }
    }

    // Sometimes, a mesh will fall outside the area that is currently visible to
    // the surface observer. In this code sample, we "sleep" any meshes that are
    // not included in the surface collection to avoid rendering them.
    // The system can including them in the collection again later, in which case
    // they will no longer be hidden.
    m_meshRenderer->HideInactiveMeshes(surfaceCollection);
}

// Updates the application state once per frame.
HolographicFrame^ HolographicSpatialMappingMain::Update()
{
    // Before doing the timer update, there is some work to do per-frame
    // to maintain holographic rendering. First, we will get information
    // about the current frame.

    // The HolographicFrame has information that the app needs in order
    // to update and render the current frame. The app begins each new
    // frame by calling CreateNextFrame.
    HolographicFrame^ holographicFrame = m_holographicSpace->CreateNextFrame();

    // Get a prediction of where holographic cameras will be when this frame
    // is presented.
    HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;

    // Back buffers can change from frame to frame. Validate each buffer, and recreate
    // resource views and depth buffers as needed.
    m_deviceResources->EnsureCameraResources(holographicFrame, prediction);

    // Next, we get a coordinate system from the attached frame of reference that is
    // associated with the current frame. Later, this coordinate system is used for
    // for creating the stereo view matrices when rendering the sample content.
    SpatialCoordinateSystem^ currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);
	
	if (connecting) {
		m_spatialAnchorHelper->ClearAnchorStore();
		m_spatialId = 0;
		connecting = false;
	}
    
	//new rendered image is requested
	if ((ack || ack3 || (m_depthStreamMode)) && !m_positionLost) {
		if (ack || ack3)m_renderAndSend = true;
		//m_spatialId = 0;
		auto cameraPose = prediction->CameraPoses->GetAt(0);
		Platform::IBox<HolographicStereoTransform>^ viewTransformContainer = cameraPose->TryGetViewTransform(currentCoordinateSystem);
		ViewProjectionConstantBuffer viewProjectionConstantBufferData;
		bool viewTransformAcquired = viewTransformContainer != nullptr;
		if (viewTransformAcquired)
		{
			HolographicStereoTransform viewCoordinateSystemTransform = viewTransformContainer->Value;
			float4x4 camPosition = viewCoordinateSystemTransform.Left;
			float4x4 viewInverse;
			bool invertible = Windows::Foundation::Numerics::invert(camPosition, &viewInverse);
			if (invertible)
			{
				//				float xp = viewInverse.m41, yp = viewInverse.m42, zp = viewInverse.m43;
				const float3 campos(viewInverse.m41, viewInverse.m42, viewInverse.m43);
				float rad = sqrt(viewInverse.m31*viewInverse.m31 + viewInverse.m33*viewInverse.m33);
				float theta = acos(viewInverse.m33 / rad);
				theta = viewInverse.m31 < 0 ? -theta : theta;
				const float3 camdirection(viewInverse.m31 / rad, 0, viewInverse.m33 / rad);
				const quaternion q(0, sin(theta / 2), 0, cos(theta / 2));
				SpatialAnchor^ anchor = SpatialAnchor::TryCreateRelativeTo(currentCoordinateSystem, campos, q);
				//m_uniqueSpatialAnchor = anchor;
				if ((anchor != nullptr) && (m_spatialAnchorHelper != nullptr))
				{
					if (ack3)m_spatialId++;
					if (ack) {
						m_spatialAnchorHelper->ClearAnchorStore();
						m_spatialId = 0;
						m_recovering = false;
					}
					// In this example, we store the anchor in an IMap.
					auto anchorMap = m_spatialAnchorHelper->GetAnchorMap();

					// Create an identifier for the anchor.
					std::wstringstream ss;
					ss << "anchor_" << m_spatialId;
					std::wstring w_char = ss.str();

					m_newKey = ref new String(w_char.c_str());

					SaveAppState();
					if (ack) {
						m_baseAnchor = anchor;
						currentCoordinateSystem = m_baseAnchor->CoordinateSystem;
						anchorMap->Insert(m_newKey->ToString(), anchor);
					}
					else if (ack3) {
						m_nextAnchor = anchor;
						currentCoordinateSystem = m_nextAnchor->CoordinateSystem;
					}
					else {
						m_anchor = anchor;
						currentCoordinateSystem = m_anchor->CoordinateSystem;
					}
				}
			}

		}
		ack = false;
		ack3 = false;
	}
	
	SpatialCoordinateSystem^ stationaryCoordinateSystem = m_stationaryReferenceFrame->CoordinateSystem;
	SpatialAnchor^ anchor;

	if (m_surfaceObserver == nullptr)
    {
        // Initialize the Surface Observer using a valid coordinate system.
        if (!m_spatialPerceptionAccessRequested)
        {
            // The spatial mapping API reads information about the user's environment. The user must
            // grant permission to the app to use this capability of the Windows Holographic device.
            auto initSurfaceObserverTask = create_task(SpatialSurfaceObserver::RequestAccessAsync());
            initSurfaceObserverTask.then([this, currentCoordinateSystem](Windows::Perception::Spatial::SpatialPerceptionAccessStatus status)
            {
                switch (status)
                {
                case SpatialPerceptionAccessStatus::Allowed:
                    m_surfaceAccessAllowed = true;
                    break;
                default:
                    // Access was denied. This usually happens because your AppX manifest file does not declare the
                    // spatialPerception capability.
                    // For info on what else can cause this, see: http://msdn.microsoft.com/library/windows/apps/mt621422.aspx
                    m_surfaceAccessAllowed = false;
                    break;
                }
            });

            m_spatialPerceptionAccessRequested = true;
        }
    }

    if (m_surfaceAccessAllowed)
    {
        SpatialBoundingBox axisAlignedBoundingBox =
        {
            {  0.f,  0.f, 0.f },
            { 20.f, 20.f, 5.f },
        };
        SpatialBoundingVolume^ bounds = SpatialBoundingVolume::FromBox(currentCoordinateSystem, axisAlignedBoundingBox);

        // If status is Allowed, we can create the surface observer.
        if (m_surfaceObserver == nullptr)
        {
            // First, we'll set up the surface observer to use our preferred data formats.
            // In this example, a "preferred" format is chosen that is compatible with our precompiled shader pipeline.
            m_surfaceMeshOptions = ref new SpatialSurfaceMeshOptions();
            IVectorView<DirectXPixelFormat>^ supportedVertexPositionFormats = m_surfaceMeshOptions->SupportedVertexPositionFormats;
            unsigned int formatIndex = 0;
            if (supportedVertexPositionFormats->IndexOf(DirectXPixelFormat::R16G16B16A16IntNormalized, &formatIndex))
            {
                m_surfaceMeshOptions->VertexPositionFormat = DirectXPixelFormat::R16G16B16A16IntNormalized;
            }
            IVectorView<DirectXPixelFormat>^ supportedVertexNormalFormats = m_surfaceMeshOptions->SupportedVertexNormalFormats;
            if (supportedVertexNormalFormats->IndexOf(DirectXPixelFormat::R8G8B8A8IntNormalized, &formatIndex))
            {
                m_surfaceMeshOptions->VertexNormalFormat = DirectXPixelFormat::R8G8B8A8IntNormalized;
            }

            // If you are using a very high detail setting with spatial mapping, it can be beneficial
            // to use a 32-bit unsigned integer format for indices instead of the default 16-bit. 
            // Uncomment the following code to enable 32-bit indices.
            //IVectorView<DirectXPixelFormat>^ supportedTriangleIndexFormats = m_surfaceMeshOptions->SupportedTriangleIndexFormats;
            //if (supportedTriangleIndexFormats->IndexOf(DirectXPixelFormat::R32UInt, &formatIndex))
            //{
            //    m_surfaceMeshOptions->TriangleIndexFormat = DirectXPixelFormat::R32UInt;
            //}

            // Create the observer.
            m_surfaceObserver = ref new SpatialSurfaceObserver();
            if (m_surfaceObserver)
            {
                m_surfaceObserver->SetBoundingVolume(bounds);

                // If the surface observer was successfully created, we can initialize our
                // collection by pulling the current data set.
                auto mapContainingSurfaceCollection = m_surfaceObserver->GetObservedSurfaces();
                for (auto const& pair : mapContainingSurfaceCollection)
                {
                    // Store the ID and metadata for each surface.
                    auto const& id = pair->Key;
                    auto const& surfaceInfo = pair->Value;
                    m_meshRenderer->AddSurface(id, surfaceInfo);
                }

                // We then subcribe to an event to receive up-to-date data.
                m_surfacesChangedToken = m_surfaceObserver->ObservedSurfacesChanged += 
                    ref new TypedEventHandler<SpatialSurfaceObserver^, Platform::Object^>(
                        bind(&HolographicSpatialMappingMain::OnSurfacesChanged, this, _1, _2)
                        );
            }
        }

        // Keep the surface observer positioned at the device's location.
        m_surfaceObserver->SetBoundingVolume(bounds);

        // Note that it is possible to set multiple bounding volumes. Pseudocode:
        //     m_surfaceObserver->SetBoundingVolumes(/* iterable collection of bounding volumes*/);
        //
        // It is also possible to use other bounding shapes - such as a view frustum. Pseudocode:
        //     SpatialBoundingVolume^ bounds = SpatialBoundingVolume::FromFrustum(coordinateSystem, viewFrustum);
        //     m_surfaceObserver->SetBoundingVolume(bounds);
    }

#ifdef DRAW_SAMPLE_CONTENT
	// Check for new input state since the last frame.
	SpatialInteractionSourceState^ pointerState = m_spatialInputHandler->CheckForInput();
	if (pointerState != nullptr)
	{
		// When a Pressed gesture is detected, the rendering mode will be changed to wireframe.

		m_drawWireframe = !m_drawWireframe;
		SpatialPointerPose^ pointerPose = pointerState->TryGetPointerPose(stationaryCoordinateSystem);
	}
	if (posfailed) {
		m_spatialId--;
		m_nextAnchor = nullptr;
		posfailed = false;
	}
	//Registering new anchor after finishing alignment
	if (ack2) {
		auto anchorMap = m_spatialAnchorHelper->GetAnchorMap();
		anchorMap->Insert(m_newKey->ToString(), m_nextAnchor);
		m_baseAnchor = m_nextAnchor;
		m_nextAnchor = nullptr;
		ack2 = false;
	}
#endif

    m_timer.Tick([&] ()
    {
#ifdef DRAW_SAMPLE_CONTENT
        m_meshRenderer->Update(m_timer, currentCoordinateSystem);
#endif
    });

	if (writer != nullptr) {


		auto cameraPose = prediction->CameraPoses->GetAt(0);
		Platform::IBox<HolographicStereoTransform>^ viewTransformContainer = cameraPose->TryGetViewTransform(stationaryCoordinateSystem);
		ViewProjectionConstantBuffer viewProjectionConstantBufferData;
		bool viewTransformAcquired = viewTransformContainer != nullptr;
		if (viewTransformAcquired)//get Camera pose in m_baseAnchor coordinate system, 
		{
			HolographicStereoTransform viewCoordinateSystemTransform = viewTransformContainer->Value;
			float4x4 camPosition = viewCoordinateSystemTransform.Left;
			float4x4 viewInverse;
			bool posLost = false;
			if (m_baseAnchor != nullptr) {
				float4x4 anchorSpaceToCurrentCoordinateSystem;
				SpatialCoordinateSystem^ anchorSpace = m_baseAnchor->CoordinateSystem;
				const auto tryTransform = anchorSpace->TryGetTransformTo(stationaryCoordinateSystem);
				if (tryTransform != nullptr)
				{
					anchorSpaceToCurrentCoordinateSystem = tryTransform->Value;
					camPosition = anchorSpaceToCurrentCoordinateSystem* camPosition;//->camPotition: base coordinates(spatial anchor coordinates)2camera coordinates 

				}
				else posLost = true;

			}

			bool invertible = Windows::Foundation::Numerics::invert(camPosition, &viewInverse);
			if (invertible)
			{

				float xp = viewInverse.m41, yp = viewInverse.m42, zp = viewInverse.m43;
				float thres = 16;
				if (m_baseAnchor != nullptr && ((m_nextAnchor == nullptr && xp*xp + yp*yp + zp*zp > thres) || m_recovering)) {
					//get anchor map
					thres = m_recovering ? thres * 2 : thres;
					auto anchorMap = m_spatialAnchorHelper->GetAnchorMap();
					double minimumerr = 100;
					IKeyValuePair<String^, SpatialAnchor^>^ bestPair;
					for each(auto& pair in anchorMap) {
						SpatialAnchor^ candidateAnchor = pair->Value;
						float4x4 anchorSpaceToCurrentCoordinateSystem;
						SpatialCoordinateSystem^ anchorSpace = candidateAnchor->CoordinateSystem;
						const auto tryTransform = anchorSpace->TryGetTransformTo(stationaryCoordinateSystem);
						float4x4 camPos_inAnchor;
						if (tryTransform != nullptr)
						{
							anchorSpaceToCurrentCoordinateSystem = tryTransform->Value;
							camPos_inAnchor = anchorSpaceToCurrentCoordinateSystem* viewCoordinateSystemTransform.Left;//->camPotition: base coordinates(spatial anchor coordinates)2camera coordinates 	
							float4x4 camPosInv;
							bool invertible_ = Windows::Foundation::Numerics::invert(camPos_inAnchor, &camPosInv);
							if (invertible_) {
								float xp = camPosInv.m41, yp = camPosInv.m42, zp = camPosInv.m43;
								double err = xp*xp + yp*yp + zp*zp;
								if (err < minimumerr) {
									bestPair = pair;
									minimumerr = err;
								}
							}
						}
					}
					if (minimumerr < thres) {
						m_baseAnchor = bestPair->Value;
						writer->WriteUInt32(HEADER_CHANC);
						Sockets::StreamSocket^ socket_ = sock;
						String^ stringToSend = bestPair->Key;
						writer->WriteInt32(writer->MeasureString(stringToSend));
						writer->WriteString(stringToSend);
						m_recovering = false;
						create_task(writer->StoreAsync()).then([this, socket_](task<unsigned int> writeTask)
						{
							try
							{
								// Try getting an exception.
								writeTask.get();
							}
							catch (Exception^ exception)
							{
								writer = nullptr;
								reader = nullptr;
							}
						});
					}
					else {
						if (!m_recovering)ack3 = true;
					}
				}
				if (posLost) {
					writer->WriteUInt32(HEADER_POSLOST);
					Sockets::StreamSocket^ socket_ = sock;
					create_task(writer->StoreAsync()).then([this, socket_](task<unsigned int> writeTask)
					{
						try
						{
							// Try getting an exception.
							writeTask.get();
						}
						catch (Exception^ exception)
						{
							writer = nullptr;
							reader = nullptr;
						}
					});

				}
				else if ((!m_depthStreamMode&&m_nextAnchor == nullptr) || (m_depthStreamMode && (checkReceive && m_nextAnchor == nullptr))) {
					checkReceive = false;
					m_depthReceived = true;
					writer->WriteUInt32(HEADER_CAMERA);
					Sockets::StreamSocket^ socket_ = sock;
					Platform::Array<unsigned char>^ buffer = ref new Platform::Array<unsigned char>(sizeof(float) * 16);
					float datamat[16] = { viewInverse.m11,viewInverse.m12,viewInverse.m13,viewInverse.m14,
						viewInverse.m21,viewInverse.m22,viewInverse.m23,viewInverse.m24,
						viewInverse.m31,viewInverse.m32,viewInverse.m33,viewInverse.m34,
						viewInverse.m41,viewInverse.m42,viewInverse.m43,viewInverse.m44, };
					void* p = datamat;
					memcpy(buffer->Data, p, sizeof(float) * 16);
					writer->WriteBytes(buffer);

					create_task(writer->StoreAsync()).then([this, socket_](task<unsigned int> writeTask)
					{
						try
						{
							// Try getting an exception.
							writeTask.get();
						}
						catch (Exception^ exception)
						{
							writer = nullptr;
							reader = nullptr;
						}
					});
				}
			}
			// Write first the length of the string a UINT32 value followed up by the string. The operation will just store 
			// the data locally.

		}
		else {//tracking lost status
			writer->WriteUInt32(HEADER_TRACKLOST);
			Sockets::StreamSocket^ socket_ = sock;
			create_task(writer->StoreAsync()).then([this, socket_](task<unsigned int> writeTask)
			{
				try
				{
					// Try getting an exception.
					writeTask.get();
				}
				catch (Exception^ exception)
				{
					writer = nullptr;
					reader = nullptr;
				}
			});


		}
	}
    // This sample uses default image stabilization settings, and does not set the focus point.

    // The holographic frame will be used to get up-to-date view and projection matrices and
    // to present the swap chain.
    return holographicFrame;
}

// Renders the current frame to each holographic camera, according to the
// current application and spatial positioning state. Returns true if the
// frame was rendered to at least one camera.
bool HolographicSpatialMappingMain::Render(
    HolographicFrame^ holographicFrame)
{
    // Don't try to render anything before the first Update.
    if (m_timer.GetFrameCount() == 0)
    {
        return false;
    }

    // Lock the set of holographic camera resources, then draw to each camera
    // in this frame.
    return m_deviceResources->UseHolographicCameraResources<bool>(
        [this, holographicFrame](std::map<UINT32, std::unique_ptr<DX::CameraResources>>& cameraResourceMap)
    {
        // Up-to-date frame predictions enhance the effectiveness of image stablization and
        // allow more accurate positioning of holograms.
        holographicFrame->UpdateCurrentPrediction();
        HolographicFramePrediction^ prediction = holographicFrame->CurrentPrediction;
        SpatialCoordinateSystem^ currentCoordinateSystem = m_referenceFrame->GetStationaryCoordinateSystemAtTimestamp(prediction->Timestamp);

        bool atLeastOneCameraRendered = false;
        for (auto cameraPose : prediction->CameraPoses)
        {
            // This represents the device-based resources for a HolographicCamera.
            DX::CameraResources* pCameraResources = cameraResourceMap[cameraPose->HolographicCamera->Id].get();

            // Get the device context.
            const auto context = m_deviceResources->GetD3DDeviceContext();
            const auto depthStencilView = pCameraResources->GetDepthStencilView();
			const auto osdepthStencilView = pCameraResources->GetOffScreenDepthStencilView();

            // Set render targets to the current holographic camera.
            ID3D11RenderTargetView *const targets[1] = { pCameraResources->GetBackBufferRenderTargetView() };
            context->OMSetRenderTargets(1, targets, depthStencilView);

            // Clear the back buffer and depth stencil view.
            context->ClearRenderTargetView(targets[0], DirectX::Colors::Transparent);
            context->ClearDepthStencilView(depthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

            // The view and projection matrices for each holographic camera will change
            // every frame. This function refreshes the data in the constant buffer for
            // the holographic camera indicated by cameraPose.
            pCameraResources->UpdateViewProjectionBuffer(m_deviceResources, cameraPose, currentCoordinateSystem);

            // Attach the view/projection constant buffer for this camera to the graphics pipeline.
            bool cameraActive = pCameraResources->AttachViewProjectionBuffer(m_deviceResources);

#ifdef DRAW_SAMPLE_CONTENT
            // Only render world-locked content when positional tracking is active.
            if (cameraActive)
            {
                // Draw the sample hologram.
                m_meshRenderer->Render(pCameraResources->IsRenderingStereoscopic(), m_drawWireframe);
            }
#endif
            atLeastOneCameraRendered = true;
			// off screen buffer 
			if ((m_renderAndSend || height_requested ||(m_depthStreamMode&&m_depthReceived)) && !m_positionLost) {
				ID3D11RenderTargetView *const targeto[1] = { pCameraResources->GetOffScreenRenderTargetView() };
				context->OMSetRenderTargets(1, targeto, osdepthStencilView);

				// Clear the back buffer and depth stencil view.
				context->ClearRenderTargetView(targeto[0], DirectX::Colors::Gray);
				context->ClearDepthStencilView(osdepthStencilView, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

				// The view and projection matrices for each holographic camera will change
				// every frame. This function refreshes the data in the constant buffer for
				// the holographic camera indicated by cameraPose.
				pCameraResources->UpdateViewProjectionBuffer_(m_deviceResources, cameraPose, currentCoordinateSystem);

				if (cameraActive)
				{
					m_meshRenderer->OffRender();
				}


				D3D11_MAPPED_SUBRESOURCE* mapped = new D3D11_MAPPED_SUBRESOURCE();
				ID3D11Texture2D* renderedTexture = pCameraResources->GetOffTexture2D();
				ID3D11Texture2D* textureBuf = pCameraResources->GetCopiedTexture2D();
				D3D11_TEXTURE2D_DESC *originDesc = new D3D11_TEXTURE2D_DESC();

				textureBuf->GetDesc(originDesc);
				int Width = originDesc->Width;
				int Height = originDesc->Height;
				
				m_deviceResources->GetD3DDeviceContext()->CopyResource(textureBuf, renderedTexture);
				DX::ThrowIfFailed(
					m_deviceResources->GetD3DDeviceContext()->Map(textureBuf, 0, D3D11_MAP_READ, 0, mapped)
				);
				
				Platform::Array<unsigned char>^ buffer = ref new Platform::Array<unsigned char>(mapped->RowPitch*Height);
				memcpy(buffer->Data, mapped->pData, mapped->RowPitch*Height);
				
				//obtain HoloLens height from floor
				Eigen::Vector3d floorpt;
				double HoloHeight;
				FloorDetection(buffer, mapped->RowPitch, Height, pCameraResources->getScale(), HoloHeight,floorpt);
				

				if (writer != nullptr) {//send image
					if (height_requested) {
						writer->WriteUInt32(HOLOLENS_HEIGHT);
						float bufData[4];
						bufData[0]=HoloHeight;	bufData[1] = floorpt(0);	bufData[2] = floorpt(1);	bufData[3] = floorpt(2);
						Sockets::StreamSocket^ socket_ = sock;
						Platform::Array<unsigned char>^ bufferheight = ref new Platform::Array<unsigned char>(sizeof(float) * 4);
						char* p = (char*)bufData;
						memcpy(bufferheight->Data, p, sizeof(float) * 4);
						writer->WriteBytes(bufferheight);

						create_task(writer->StoreAsync()).then([this, socket_](task<unsigned int> writeTask)
						{
							try
							{
								// Try getting an exception.
								writeTask.get();
							}
							catch (Exception^ exception)
							{
								writer = nullptr;
								reader = nullptr;
							}
						});
					}
					
					if (m_renderAndSend || (m_depthStreamMode&&m_depthReceived)) {
						if (m_renderAndSend) {
							m_renderAndSend = false;
							writer->WriteUInt32(HEADER_IMAGE);
							String^ stringToSend = m_newKey;
							writer->WriteInt32(writer->MeasureString(stringToSend));
							writer->WriteString(stringToSend);
						}
						else {
							writer->WriteUInt32(HEADER_DEPTHSTREAM);
							m_depthReceived = false;
						}

						writer->WriteUInt32(mapped->RowPitch);
						writer->WriteUInt32(Height);
						writer->WriteBytes(buffer);
						delete buffer;
						delete mapped;
						Sockets::StreamSocket^ socket_ = sock;

						create_task(writer->StoreAsync()).then([this, socket_](task<unsigned int> writeTask)
						{
							try
							{
								// Try getting an exception.
								writeTask.get();
							}
							catch (Exception^ exception)
							{
								writer = nullptr;
								reader = nullptr;
							}
						});
					}

				}
				else {
					delete buffer;
					delete mapped;

				}

			}
			else if (m_positionLost) {
				m_nextAnchor = nullptr;
			}
        }

        return atLeastOneCameraRendered;
    });
}

void HolographicSpatialMappingMain::SaveAppState()
{
    // This sample does not persist any state between sessions.
}

void HolographicSpatialMappingMain::LoadAppState()
{
	if (m_spatialAnchorHelper != nullptr)
	{
		m_spatialAnchorHelper->TrySaveToAnchorStore();
	}
}

// Notifies classes that use Direct3D device resources that the device resources
// need to be released before this method returns.
void HolographicSpatialMappingMain::OnDeviceLost()
{
#ifdef DRAW_SAMPLE_CONTENT
    m_meshRenderer->ReleaseDeviceDependentResources();
#endif
	SaveAppState();
}

// Notifies classes that use Direct3D device resources that the device resources
// may now be recreated.
void HolographicSpatialMappingMain::OnDeviceRestored()
{
#ifdef DRAW_SAMPLE_CONTENT
    m_meshRenderer->CreateDeviceDependentResources();
#endif
}

void HolographicSpatialMappingMain::OnPositionalTrackingDeactivating(
    SpatialLocator^ sender, 
    SpatialLocatorPositionalTrackingDeactivatingEventArgs^ args)
{
    // Without positional tracking, spatial meshes will not be locatable.
    args->Canceled = true;
}

void HolographicSpatialMappingMain::OnCameraAdded(
    HolographicSpace^ sender,
    HolographicSpaceCameraAddedEventArgs^ args)
{
    Deferral^ deferral = args->GetDeferral();
    HolographicCamera^ holographicCamera = args->Camera;
    create_task([this, deferral, holographicCamera] ()
    {
        // Create device-based resources for the holographic camera and add it to the list of
        // cameras used for updates and rendering. Notes:
        //   * Since this function may be called at any time, the AddHolographicCamera function
        //     waits until it can get a lock on the set of holographic camera resources before
        //     adding the new camera. At 60 frames per second this wait should not take long.
        //   * A subsequent Update will take the back buffer from the RenderingParameters of this
        //     camera's CameraPose and use it to create the ID3D11RenderTargetView for this camera.
        //     Content can then be rendered for the HolographicCamera.
        m_deviceResources->AddHolographicCamera(holographicCamera);

        // Holographic frame predictions will not include any information about this camera until
        // the deferral is completed.
        deferral->Complete();
    });
	LoadAnchorStore();
}

void HolographicSpatialMappingMain::OnCameraRemoved(
    HolographicSpace^ sender,
    HolographicSpaceCameraRemovedEventArgs^ args)
{
    // Before letting this callback return, ensure that all references to the back buffer
    // are released.
    // Since this function may be called at any time, the RemoveHolographicCamera function
    // waits until it can get a lock on the set of holographic camera resources before
    // deallocating resources for this camera. At 60 frames per second this wait should
    // not take long.
    m_deviceResources->RemoveHolographicCamera(args->Camera);
}

void HolographicSpatialMappingMain::LoadAnchorStore() {

	m_spatialAnchorHelper = create_task(SpatialAnchorManager::RequestStoreAsync())
		.then([](task<SpatialAnchorStore^> previousTask)
	{
		std::shared_ptr<SampleSpatialAnchorHelper> newHelper = nullptr;


		try
		{
			SpatialAnchorStore^ anchorStore = previousTask.get();

			// Once the SpatialAnchorStore has been loaded by the system, we can create our helper class.

			// Using "new" to access private constructor
			newHelper = std::shared_ptr<SampleSpatialAnchorHelper>(new SampleSpatialAnchorHelper(anchorStore));


			newHelper->LoadFromAnchorStore();

			// Now we can load anchors from the store.

		}
		catch (Exception^ exception)
		{
			/*		PrintWstringToDebugConsole(
			std::wstring(L"Exception while loading the anchor store: ") +
			exception->Message->Data() +
			L"\n"
			);*/
		}

		// Return the initialized class instance.
		return newHelper;
	}).get();

}

HolographicSpatialMappingMain::SampleSpatialAnchorHelper::SampleSpatialAnchorHelper(SpatialAnchorStore^ anchorStore)
{
	m_anchorStore = anchorStore;
	m_anchorMap = ref new Platform::Collections::Map<String^, SpatialAnchor^>();
}

bool HolographicSpatialMappingMain::SampleSpatialAnchorHelper::TrySaveToAnchorStore()
{
	// This function returns true if all the anchors in the in-memory collection are saved to the anchor
	// store. If zero anchors are in the in-memory collection, we will still return true because the
	// condition has been met.
	bool success = true;

	// If access is denied, 'anchorStore' will not be obtained.
	if (m_anchorStore != nullptr)
	{
		for each (auto& pair in m_anchorMap)
		{
			auto const& id = pair->Key;
			auto const& anchor = pair->Value;

			// Try to save the anchors.
			if (!m_anchorStore->TrySave(id, anchor))
			{
				// This may indicate the anchor ID is taken, or the anchor limit is reached for the app.
				success = false;
			}
		}
	}

	return success;
}

void HolographicSpatialMappingMain::SampleSpatialAnchorHelper::LoadFromAnchorStore()
{
	// If access is denied, 'anchorStore' will not be obtained.
	if (m_anchorStore != nullptr)
	{
		// Get all saved anchors.
		auto anchorMapView = m_anchorStore->GetAllSavedAnchors();
		for each (auto const& pair in anchorMapView)
		{
			auto const& id = pair->Key;
			auto const& anchor = pair->Value;
			m_anchorMap->Insert(id, anchor);
		}
	}
}

void HolographicSpatialMappingMain::SampleSpatialAnchorHelper::ClearAnchorStore()
{
	// If access is denied, 'anchorStore' will not be obtained.
	if (m_anchorStore != nullptr)
	{
		// Clear all anchors from the store.
		m_anchorMap->Clear();
		m_anchorStore->Clear();
	}
}

void HolographicSpatialMappingMain::OnLocatabilityChanged(SpatialLocator^ sender, Object^ args)
{
	String^ mes = sender->Locatability.ToString();
	switch (sender->Locatability)
	{
	case SpatialLocatability::Unavailable:
		// Holograms cannot be rendered.
	{
		String^ message = L"Warning! Positional tracking is " +
			sender->Locatability.ToString() + L".\n";
		OutputDebugStringW(message->Data());
	}
	break;

	// In the following three cases, it is still possible to place holograms using a
	// SpatialLocatorAttachedFrameOfReference.
	case SpatialLocatability::PositionalTrackingActivating:
		// The system is preparing to use positional tracking.

	case SpatialLocatability::OrientationOnly:
		// Positional tracking has not been activated.

	case SpatialLocatability::PositionalTrackingInhibited:
		// Positional tracking is temporarily inhibited. User action may be required
		// in order to restore positional tracking.
		m_positionLost = true;
		break;

	case SpatialLocatability::PositionalTrackingActive:
		// Positional tracking is active. World-locked content can be rendered.
		m_recovering = true;
		m_positionLost = false;
		break;
	}
}