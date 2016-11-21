/*!
  @file   drishti_cereal_pba.h
  @author David Hirvonen
  @brief  Provides compatibility with boost::serialization Archive::is_loading::value

*/

#ifndef DRISHTI_CORE_CEREAL_PBA_H_
#define DRISHTI_CORE_CEREAL_PBA_H_

// http://uscilab.github.io/cereal/serialization_archives.html
//#include <cereal/archives/binary.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>

DRISHTI_BEGIN_NAMESPACE(cereal)

template< bool C_ > struct bool_
{
    static const bool value = C_;
    typedef bool_ type;
    typedef bool value_type;
    operator bool() const { return this->value; }
};

class PortableBinaryOutputArchive3 : public OutputArchive<PortableBinaryOutputArchive3, AllowEmptyClassElision>
{
public:

    typedef bool_<false> is_loading;
    typedef bool_<true> is_saving;
    
    //! Construct, outputting to the provided stream
    /*! @param stream The stream to output to.  Can be a stringstream, a file stream, or
     even cout! */
    PortableBinaryOutputArchive3(std::ostream & stream) :
    OutputArchive<PortableBinaryOutputArchive3, AllowEmptyClassElision>(this),
    itsStream(stream)
    {
        this->operator()( portable_binary_detail::is_little_endian() );
    }
    
    //! Writes size bytes of data to the output stream
    void saveBinary( const void * data, std::size_t size )
    {
        auto const writtenSize = static_cast<std::size_t>( itsStream.rdbuf()->sputn( reinterpret_cast<const char*>( data ), size ) );
        
        if(writtenSize != size)
            throw Exception("Failed to write " + std::to_string(size) + " bytes to output stream! Wrote " + std::to_string(writtenSize));
    }
    
private:
    std::ostream & itsStream;
};

class PortableBinaryInputArchive3 : public InputArchive<PortableBinaryInputArchive3, AllowEmptyClassElision>
{
public:

    typedef bool_<true> is_loading;
    typedef bool_<false> is_saving;
    
    //! Construct, loading from the provided stream
    /*! @param stream The stream to read from. */
    PortableBinaryInputArchive3(std::istream & stream) :
    InputArchive<PortableBinaryInputArchive3, AllowEmptyClassElision>(this),
    itsStream(stream),
    itsConvertEndianness( false )
    {
        bool streamLittleEndian;
        this->operator()( streamLittleEndian );
        itsConvertEndianness = portable_binary_detail::is_little_endian() ^ streamLittleEndian;
    }
    
    //! Reads size bytes of data from the input stream
    /*! @param data The data to save
     @param size The number of bytes in the data
     @tparam DataSize T The size of the actual type of the data elements being loaded */
    template <std::size_t DataSize>
    void loadBinary( void * const data, std::size_t size )
    {
        // load data
        auto const readSize = static_cast<std::size_t>( itsStream.rdbuf()->sgetn( reinterpret_cast<char*>( data ), size ) );
        
        if(readSize != size)
            throw Exception("Failed to read " + std::to_string(size) + " bytes from input stream! Read " + std::to_string(readSize));
        
        // flip bits if needed
        if( itsConvertEndianness )
        {
            std::uint8_t * ptr = reinterpret_cast<std::uint8_t*>( data );
            for( std::size_t i = 0; i < size; i += DataSize )
                portable_binary_detail::swap_bytes<DataSize>( ptr );
        }
    }
    
private:
    std::istream & itsStream;
    bool itsConvertEndianness; //!< If set to true, we will need to swap bytes upon loading
};


// ######################################################################
// Common BinaryArchive serialization functions

//! Saving for POD types to portable binary
template<class T> inline
typename std::enable_if<std::is_arithmetic<T>::value, void>::type
CEREAL_SAVE_FUNCTION_NAME(PortableBinaryOutputArchive3 & ar, T const & t)
{
    static_assert( !std::is_floating_point<T>::value ||
                  (std::is_floating_point<T>::value && std::numeric_limits<T>::is_iec559),
                  "Portable binary only supports IEEE 754 standardized floating point" );
    ar.saveBinary(std::addressof(t), sizeof(t));
}

//! Loading for POD types from portable binary
template<class T> inline
typename std::enable_if<std::is_arithmetic<T>::value, void>::type
CEREAL_LOAD_FUNCTION_NAME(PortableBinaryInputArchive3 & ar, T & t)
{
    static_assert( !std::is_floating_point<T>::value ||
                  (std::is_floating_point<T>::value && std::numeric_limits<T>::is_iec559),
                  "Portable binary only supports IEEE 754 standardized floating point" );
    ar.template loadBinary<sizeof(T)>(std::addressof(t), sizeof(t));
}

//! Serializing NVP types to portable binary
template <class Archive, class T> inline
CEREAL_ARCHIVE_RESTRICT(PortableBinaryInputArchive3, PortableBinaryOutputArchive3)
CEREAL_SERIALIZE_FUNCTION_NAME( Archive & ar, NameValuePair<T> & t )
{
    ar( t.value );
}

//! Serializing SizeTags to portable binary
template <class Archive, class T> inline
CEREAL_ARCHIVE_RESTRICT(PortableBinaryInputArchive3, PortableBinaryOutputArchive3)
CEREAL_SERIALIZE_FUNCTION_NAME( Archive & ar, SizeTag<T> & t )
{
    ar( t.size );
}

//! Saving binary data to portable binary
template <class T> inline
void CEREAL_SAVE_FUNCTION_NAME(PortableBinaryOutputArchive3 & ar, BinaryData<T> const & bd)
{
    typedef typename std::remove_pointer<T>::type TT;
    static_assert( !std::is_floating_point<TT>::value ||
                  (std::is_floating_point<TT>::value && std::numeric_limits<TT>::is_iec559),
                  "Portable binary only supports IEEE 754 standardized floating point" );
    
    ar.saveBinary( bd.data, static_cast<std::size_t>( bd.size ) );
}

//! Loading binary data from portable binary
template <class T> inline
void CEREAL_LOAD_FUNCTION_NAME(PortableBinaryInputArchive3 & ar, BinaryData<T> & bd)
{
    typedef typename std::remove_pointer<T>::type TT;
    static_assert( !std::is_floating_point<TT>::value ||
                  (std::is_floating_point<TT>::value && std::numeric_limits<TT>::is_iec559),
                  "Portable binary only supports IEEE 754 standardized floating point" );
    
    ar.template loadBinary<sizeof(TT)>( bd.data, static_cast<std::size_t>( bd.size ) );
}

DRISHTI_END_NAMESPACE(cereal)

// register archives for polymorphic support
CEREAL_REGISTER_ARCHIVE(cereal::PortableBinaryOutputArchive3)
CEREAL_REGISTER_ARCHIVE(cereal::PortableBinaryInputArchive3)

// tie input and output archives together
CEREAL_SETUP_ARCHIVE_TRAITS(cereal::PortableBinaryInputArchive3, cereal::PortableBinaryOutputArchive3)



#endif // DRISHTI_CORE_CEREAL_PBA_H_
